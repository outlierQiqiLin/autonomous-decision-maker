#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ml_online_learning_node.py
============================================================
[Purpose]
- Implements the “explore → update → query again” online-learning loop:
  1) Initial prior: for any (target object, location category), the default “find probability” is 0.5
  2) After each exploration, an external node feeds (object, location, found) to this node
  3) This node accumulates success/failure counts and updates the probability with a prior
  4) When the knowledge base makes candidate decisions, it queries this node for probabilities to guide ranking

[Probability model]
- For each (obj, loc_type), maintain Beta-Bernoulli counts:
    success = S, failure = F
- Prior: alpha=1, beta=1  => initial mean alpha/(alpha+beta)=0.5
- For each observation:
    found=True  => S += 1
    found=False => F += 1
- Output probability (posterior mean):
    p = (alpha + S) / (alpha + beta + S + F)
"""

import os
import csv
import pickle
import threading
from dataclasses import dataclass, asdict
from typing import Dict, Tuple, List

from world_percept_assig4.srv import PredictLikelihood, PredictLikelihoodResponse
from world_percept_assig4.srv import AddObservation, AddObservationResponse


import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String


# -----------------------------
# 1) Core: Probability memory
# -----------------------------
@dataclass
class BetaCounter:
    """Success / failure counts for a specific (obj, loc_type) pair"""
    success: int = 0
    failure: int = 0


class ProbabilityMemory:
    """
    Maintains Beta counts for each (object, location_type) pair
    and outputs the posterior mean probability.
    Initial prior: alpha=beta=1 => p=0.5
    """
    def __init__(self, alpha: float = 1.0, beta: float = 1.0):
        self.alpha = float(alpha)
        self.beta = float(beta)
        self._mem: Dict[Tuple[str, str], BetaCounter] = {}
        self._lock = threading.Lock()

    @staticmethod
    def _norm(s: str) -> str:
        if s is None:
            return "unknown"
        return s.strip().lower()

    def update(self, obj: str, loc_type: str, found: bool) -> None:
        obj = self._norm(obj)
        loc_type = self._norm(loc_type)
        key = (obj, loc_type)
        with self._lock:
            if key not in self._mem:
                self._mem[key] = BetaCounter()
            if found:
                self._mem[key].success += 1
            else:
                self._mem[key].failure += 1

    def probability(self, obj: str, loc_type: str) -> float:
        obj = self._norm(obj)
        loc_type = self._norm(loc_type)
        key = (obj, loc_type)
        with self._lock:
            c = self._mem.get(key, BetaCounter())
            s, f = c.success, c.failure
        # Posterior mean
        return float((self.alpha + s) / (self.alpha + self.beta + s + f))

    def snapshot_for_object(self, obj: str, loc_types: List[str]) -> List[Tuple[str, float, int, int]]:
        """Return a list of (loc_type, p, S, F) for a given object over multiple location types"""
        obj = self._norm(obj)
        out = []
        with self._lock:
            for lt in loc_types:
                lt2 = self._norm(lt)
                c = self._mem.get((obj, lt2), BetaCounter())
                p = (self.alpha + c.success) / (self.alpha + self.beta + c.success + c.failure)
                out.append((lt, float(p), c.success, c.failure))
        return out

    def dump_state(self) -> dict:
        """For persistence: convert memory into a pickle-serializable dictionary"""
        with self._lock:
            mem_dump = { (k[0], k[1]) : asdict(v) for k, v in self._mem.items() }
        return {
            "alpha": self.alpha,
            "beta": self.beta,
            "mem": mem_dump
        }

    @staticmethod
    def load_state(state: dict) -> "ProbabilityMemory":
        pm = ProbabilityMemory(alpha=state.get("alpha", 1.0), beta=state.get("beta", 1.0))
        mem_dump = state.get("mem", {})
        with pm._lock:
            for (obj, lt), v in mem_dump.items():
                pm._mem[(obj, lt)] = BetaCounter(success=int(v.get("success", 0)), failure=int(v.get("failure", 0)))
        return pm


# -----------------------------
# 2) ROS node
# -----------------------------
class MLOnlineLearningNode:
    def __init__(self):
        rospy.init_node("ml_online_learning_node")

        # Four candidate locations (fixed for the current scenario)
        self.candidates_raw = [
            "book shelf",
            "cafe table",
            "table",
            "trash contain",
        ]

        # Prior：alpha=beta=1 => initial p=0.5
        self.alpha = float(rospy.get_param("~alpha", 1.0))
        self.beta = float(rospy.get_param("~beta", 1.0))

        
        default_dir = os.path.join(os.path.expanduser("~"), ".ros")
        os.makedirs(default_dir, exist_ok=True)
        self.memory_path = rospy.get_param("~memory_path", os.path.join(default_dir, "ml_prob_memory.pkl"))
        self.csv_path = rospy.get_param("~csv_path", os.path.join(default_dir, "ml_prob_history.csv"))
        self.plot_path = rospy.get_param("~plot_path", os.path.join(default_dir, "ml_prob_plot.png"))

        
        self.save_plot = bool(rospy.get_param("~save_plot", True))

        # load or init memory
        self.pm = self._load_or_init_memory()
        # -----------------------------
        # Online confusion matrix
        # -----------------------------
        self.cm_threshold = 0.5   # p >= 0.5 
        self.cm_tp = 0
        self.cm_fp = 0
        self.cm_tn = 0
        self.cm_fn = 0
        self.cm_n  = 0


        self.pub_snapshot = rospy.Publisher("/ml_node/prob_snapshot", String, queue_size=10)

        # Trigger
        self.srv_add = rospy.Service("/ml_node/add_observation", Trigger, self.handle_add_observation)
        self.srv_pred = rospy.Service("/ml_node/predict", Trigger, self.handle_predict)
        self.srv_rank = rospy.Service("/ml_node/rank_candidates", Trigger, self.handle_rank_candidates)

        # ★ Probability query service for the C++ learning_node (service name is "predict_likelihood")
        self.srv_pred_cpp = rospy.Service("predict_likelihood", PredictLikelihood, self.handle_predict_srv)

        # ★ Exploration result update service for the C++ learning_node (service name is "add_observation")
        self.srv_add_cpp = rospy.Service("add_observation", AddObservation, self.handle_add_observation_srv)
        rospy.loginfo("ML online node start.")
        rospy.loginfo("Service:/ml_node/add_observation  /ml_node/predict  /ml_node/rank_candidates")
        rospy.loginfo("memorey files:%s", self.memory_path)
        rospy.loginfo("history CSV:%s", self.csv_path)
        rospy.loginfo("plot :%s (save_plot=%s)", self.plot_path, self.save_plot)

        
        self._publish_and_log_snapshot(obj="bowl")
    # -----------------------------
    # 4) C++ interface for integration
    # -----------------------------
    def handle_predict_srv(self, req):
        """
        Probability query interface called by the C++ learning_node:
        Request: target_class, location_name
        Response: probability
        """
        obj = req.target_class
        loc_name = req.location_name
        loc_type = self.normalize_location(loc_name)

        p = self.pm.probability(obj, loc_type)
        return PredictLikelihoodResponse(p)
    
    def _update_and_print_confusion(self, p_before: float, found: bool):
    
        y_pred = 1 if p_before >= self.cm_threshold else 0
        y_true = 1 if found else 0

        if y_pred == 1 and y_true == 1:
            self.cm_tp += 1
        elif y_pred == 1 and y_true == 0:
            self.cm_fp += 1
        elif y_pred == 0 and y_true == 0:
            self.cm_tn += 1
        else:
            self.cm_fn += 1

        self.cm_n += 1

        
        acc = (self.cm_tp + self.cm_tn) / self.cm_n if self.cm_n > 0 else 0.0

        print("=== Confusion Matrix (online) ===")
        print("threshold =", self.cm_threshold)
        print("Pred\\True | found(1) | not_found(0)")
        print(f"found(1)  | TP={self.cm_tp:<4d} | FP={self.cm_fp:<4d}")
        print(f"not(0)    | FN={self.cm_fn:<4d} | TN={self.cm_tn:<4d}")
        print(f"n={self.cm_n}  accuracy={acc:.3f}")
        print()



    def handle_add_observation_srv(self, req):
        """
         Interface for writing exploration results, called by the C++ learning_node:
        Request: target_class, location_name, found
        Response: ok, message
        """
        obj = req.target_class
        loc_name = req.location_name
        found = bool(req.found)

        loc_type = self.normalize_location(loc_name)

        # ★ 1) Get the probability BEFORE updating (for the confusion matrix)
        p_before = self.pm.probability(obj, loc_type)
        self._update_and_print_confusion(p_before, found)

        # ★ 2) Perform the actual learning update
        self.pm.update(obj, loc_type, found)

        ok, save_msg = self._save_memory()

        # ★ 3) Keep the original probability bar visualization 
        self._publish_and_log_snapshot(obj=obj)


        msg = f"updated: obj={obj}, loc_name='{loc_name}' -> loc_type={loc_type}, found={found}; {save_msg}"
        rospy.loginfo(msg)

        return AddObservationResponse(ok, msg)





    # ---------- Location name normalization ----------
    def normalize_location(self, location_name: str) -> str:
        """
         Normalize any location_name into one of 4 categories (or unknown):
        - book_shelf
        - cafe_table
        - table
        - trash_container
        """
        if location_name is None:
            return "unknown"
        s = location_name.strip().lower().replace("-", "_").replace(" ", "_")

        if "book" in s and "shelf" in s:
            return "book_shelf"
        if "cafe" in s and "table" in s:
            return "cafe_table"
        if "table" in s:
            return "table"
        if "trash" in s and ("contain" in s or "container" in s or "bin" in s):
            return "trash_container"
        if "trash_container" in s:
            return "trash_container"
        if "bin" in s:
            return "trash_container"

        return "unknown"

    def candidate_loc_types(self) -> List[str]:
        
        return [self.normalize_location(x) for x in self.candidates_raw]


    def _load_or_init_memory(self) -> ProbabilityMemory:
        if os.path.exists(self.memory_path):
            try:
                with open(self.memory_path, "rb") as f:
                    state = pickle.load(f)
                pm = ProbabilityMemory.load_state(state)
                rospy.loginfo("loading history of probability")
                return pm
            except Exception as e:
                rospy.logwarn("loading history of probability failed.Reason as :%s", str(e))
        return ProbabilityMemory(alpha=self.alpha, beta=self.beta)

    def _save_memory(self) -> Tuple[bool, str]:
        try:
            with open(self.memory_path, "wb") as f:
                pickle.dump(self.pm.dump_state(), f)
            return True, "saved successfully"
        except Exception as e:
            return False, f"saved failed:{e}"

    # ---------- visualization: bar chart ----------
    @staticmethod
    def _bar(p: float, width: int = 20) -> str:
        p = max(0.0, min(1.0, float(p)))
        n = int(round(p * width))
        return "█" * n + "░" * (width - n)

    def _publish_and_log_snapshot(self, obj: str) -> None:
        """
       Generate a probability snapshot for the four candidate locations:
        - Publish to /ml_node/prob_snapshot (String)
        - Print an intuitive bar chart to the console

        """
        loc_types = self.candidate_loc_types()
        snap = self.pm.snapshot_for_object(obj, loc_types)

        # Sort (descending)
        snap_sorted = sorted(snap, key=lambda x: x[1], reverse=True)

        # Build a string message
        lines = [f"Target  object: {obj}  (Prio alpha=beta=1 => init 0.5)"]
        for lt, p, s, f in snap_sorted:
            lines.append(f"- {lt:<15s}  p={p:.4f}  S={s} F={f}  {self._bar(p)}")
        best = snap_sorted[0][0]
        msg = "\n".join(lines)

        # Publish topic
        self.pub_snapshot.publish(String(data=msg))

        # Print to console
        print("\n" + msg + "\n")

        # Append to CSV 
        self._append_csv(obj, snap)

        # Optionally save a plot image
        if self.save_plot:
            self._save_plot_png(obj)

    def _append_csv(self, obj: str, snap: List[Tuple[str, float, int, int]]) -> None:
        """
        CSV column:
        timestamp, object, loc_type, prob, success, failure
        """
        import datetime
        ts = datetime.datetime.now().isoformat(timespec="seconds")

        file_exists = os.path.exists(self.csv_path)
        try:
            with open(self.csv_path, "a", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                if not file_exists:
                    w.writerow(["timestamp", "object", "loc_type", "prob", "success", "failure"])
                for lt, p, s, ff in snap:
                    w.writerow([ts, obj, lt, f"{p:.6f}", s, ff])
        except Exception as e:
            rospy.logwarn("Failed to write CSV:%s", str(e))

    def _save_plot_png(self, obj: str) -> None:
        """
        save plot png to self.plot_path。
    
        """
        try:
            import matplotlib
            matplotlib.use("Agg") 
            import matplotlib.pyplot as plt
        except Exception as e:
            rospy.logwarn("matplotlib no install,can not save plot.%s", str(e))
            return

        loc_types = self.candidate_loc_types()
        snap = self.pm.snapshot_for_object(obj, loc_types)
        snap_sorted = sorted(snap, key=lambda x: x[1], reverse=True)

        labels = [x[0] for x in snap_sorted]
        probs = [x[1] for x in snap_sorted]

        plt.figure()
        plt.title(f"Probability of finding '{obj}' by location (Beta prior mean)")
        plt.xlabel("location_type")
        plt.ylabel("P(found)")
        plt.ylim(0.0, 1.0)
        plt.bar(labels, probs)
        plt.tight_layout()
        try:
            plt.savefig(self.plot_path, dpi=160)
        except Exception as e:
            rospy.logwarn("failed save plot:%s", str(e))
        finally:
            plt.close()

    # -----------------------------
    # 3) ROS Service ： Trigger 
    # -----------------------------
    def handle_add_observation(self, _req):
        """
       read and renew：
          ~obs_target_class  (str)  
          ~obs_location_name (str)  eg: "cafe table" / "book shelf" / ...
          ~obs_found         (bool) True/False
        """
        obj = rospy.get_param("~obs_target_class", "bowl")
        loc_name = rospy.get_param("~obs_location_name", "table")
        found = bool(rospy.get_param("~obs_found", False))

        loc_type = self.normalize_location(loc_name)

        # update memory
        self.pm.update(obj, loc_type, found)

        ok, save_msg = self._save_memory()

        self._publish_and_log_snapshot(obj=obj)

        msg = f"update observe:obj={obj}, loc_name='{loc_name}' -> loc_type={loc_type}, found={found};{save_msg}"
        rospy.loginfo(msg)

        return TriggerResponse(success=True if ok else False, message=msg)

    def handle_predict(self, _req):
        """
          ~query_target_class  (str)
          ~query_location_name (str)
        """
        obj = rospy.get_param("~query_target_class", "bowl")
        loc_name = rospy.get_param("~query_location_name", "table")
        loc_type = self.normalize_location(loc_name)

        p = self.pm.probability(obj, loc_type)

        rospy.set_param("~last_probability", float(p))
        rospy.set_param("~last_query_loc_type", loc_type)

        msg = f"P(found=1 | obj={obj}, loc_type={loc_type}) = {p:.4f}"
        return TriggerResponse(success=True, message=msg)

    def handle_rank_candidates(self, _req):
        """
          input: ~rank_target_class (str)
          output:
            - TriggerResponse.message gives best location_type
            - ~rank_result 
            - topic /ml_node/prob_snapshot 
        """
        obj = rospy.get_param("~rank_target_class", "bowl")
        loc_types = self.candidate_loc_types()
        snap = self.pm.snapshot_for_object(obj, loc_types)
        snap_sorted = sorted(snap, key=lambda x: x[1], reverse=True)

        best_loc_type, best_p, _, _ = snap_sorted[0]

        
        rank_result = [{"loc_type": lt, "prob": float(p), "success": int(s), "failure": int(f)} for lt, p, s, f in snap_sorted]
        rospy.set_param("~rank_result", rank_result)
        rospy.set_param("~rank_best_loc_type", best_loc_type)
        rospy.set_param("~rank_best_prob", float(best_p))

   
        self._publish_and_log_snapshot(obj=obj)

        msg = f"best={best_loc_type}, prob={best_p:.4f}"
        return TriggerResponse(success=True, message=msg)


if __name__ == "__main__":
    node = MLOnlineLearningNode()
    rospy.spin()

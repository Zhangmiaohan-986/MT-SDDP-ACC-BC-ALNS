import time

STATE = {
    "cbs_calls": [],
    "save_kwargs": None,
    "timed_lists": [],
}
_depth = {"cbs": 0}

class TimedList(list):

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.r12_ts = []
        STATE["timed_lists"].append(self)

    def append(self, x):
        super().append(x)
        self.r12_ts.append(time.perf_counter())

def reset():
    STATE["cbs_calls"] = []
    STATE["save_kwargs"] = None
    STATE["timed_lists"] = []

def _wrap_cbs(fn):
    def wrapped(*args, **kwargs):
        if _depth["cbs"] > 0:
            return fn(*args, **kwargs)
        _depth["cbs"] += 1
        t0 = time.perf_counter()
        try:
            return fn(*args, **kwargs)
        finally:
            STATE["cbs_calls"].append((t0, time.perf_counter()))
            _depth["cbs"] -= 1
    wrapped.__wrapped__ = fn
    wrapped.__name__ = getattr(fn, "__name__", "rolling_time_cbs")
    wrapped._r12 = True
    return wrapped

def _wrap_save(fn):
    def wrapped(*args, **kwargs):
        STATE["save_kwargs"] = dict(kwargs)
        STATE["save_args"] = args
        return fn(*args, **kwargs)
    wrapped.__wrapped__ = fn
    wrapped._r12 = True
    return wrapped

def install():
    import cbs_plan
    if not getattr(cbs_plan.rolling_time_cbs, "_r12", False):
        cbs_plan.rolling_time_cbs = _wrap_cbs(cbs_plan.rolling_time_cbs)
    import task_data
    if not getattr(task_data.save_alns_results, "_r12", False):
        task_data.save_alns_results = _wrap_save(task_data.save_alns_results)

def propagate():

    import sys
    import cbs_plan, task_data
    w_cbs = cbs_plan.rolling_time_cbs
    w_save = task_data.save_alns_results
    orig_cbs = w_cbs.__wrapped__
    orig_save = w_save.__wrapped__
    patched = []
    for name, m in list(sys.modules.items()):
        if m is None:
            continue
        d = getattr(m, "__dict__", None)
        if not d:
            continue
        if d.get("rolling_time_cbs") is orig_cbs:
            d["rolling_time_cbs"] = w_cbs; patched.append(name + ".rolling_time_cbs")
        if d.get("save_alns_results") is orig_save:
            d["save_alns_results"] = w_save; patched.append(name + ".save_alns_results")
    return patched

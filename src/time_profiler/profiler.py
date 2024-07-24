import cProfile, pstats, io
import statistics
import time
import rospy

def profiling_decorator(func):
    debug = rospy.get_param('debug', False)
    if debug == False:
        return func
    def wrapper(*args, **kwargs):
        profiler = cProfile.Profile()        
        result = profiler.runcall(func, *args, **kwargs)
        buffer = io.StringIO()
        ps = pstats.Stats(profiler, stream=buffer).strip_dirs().sort_stats('cumulative')
        ps.print_stats()
        msg = "\n" + buffer.getvalue() + "\n"
        rospy.loginfo(msg)
        # profiler.print_stats(sort='cumulative')
        return result
    return wrapper

def timing_decorator(func):
    debug = rospy.get_param('debug', False)
    if debug == False:
        return func
    avg_list = []
    def wrapper(*args, **kwargs):
        t1 = time.perf_counter()
        result = func(*args, **kwargs)
        t2 = time.perf_counter()
        t_diff = t2 - t1
        avg_list.append(t_diff)
        t_avg = statistics.fmean(avg_list if len(avg_list) <= 10 else avg_list[10:])
        t_max = max(avg_list if len(avg_list) <= 10 else avg_list[10:])
        t_min = min(avg_list if len(avg_list) <= 10 else avg_list[10:])
        msg = "\n"
        msg += f"{func.__name__}() \n"
        msg += f" Real time: {t_diff:.3f} seconds \n"
        msg += f" Mean time: {t_avg:.3f} seconds \n"
        msg += f" Min time: {t_min:.3f} seconds \n"
        msg += f" Max time: {t_max:.3f} seconds \n"
        msg += "\n"
        rospy.loginfo(msg)
        return result
    return wrapper
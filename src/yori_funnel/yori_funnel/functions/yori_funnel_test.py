import yori_funnel_functions as YORI
import time
import numpy as np

yori = YORI.yori_funnel()
# yori.funnel_fold_up()
# yori.funnel_fold_down()
# yori.funnel_fold_up()
# yori.funnel_backward()
# yori.funnel_forward()
yori.all_actions([-20, -90, -90], 100)

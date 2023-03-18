import yori_funnel_functions as YORI
import time

yori = YORI.yori_funnel()
# yori.funnel_fold_up()
# yori.funnel_fold_down()
# yori.funnel_fold_up()
# yori.funnel_backward()
# yori.funnel_forward()

yori.funnel_up_down(32)
time.sleep(1)
yori.funnel_up_down(-32)

#libraries-----------------------------------------------------------------------------
from fpioa_manager import *
from modules import ws2812
import sensor
import image
import lcd
import time
import utime
import math
import _thread
from machine import UART
#from pyb import UART
from Maix import GPIO
import KPU as kpu
#--------------------------------------------------------------------------------------
#thresholds----------------------------------------------------------------------------
#o:橙 g:緑 b:青 y:黄
thresholds_o = (37, 89, 3, 19, 27, 45)
thresholds_g = (17, 100, -128, 127, -128, 127)
thresholds_b = (21, 37, -27, 31, -64, -1)
thresholds_y = (55, 69, -34, -16, 46, 66)
#--------------------------------------------------------------------------------------
#option--------------------------------------------------------------------------------
class_ws2812 = ws2812(8, 1)
fm.register(34,fm.fpioa.UART1_RX)
fm.register(35,fm.fpioa.UART1_TX)
uart_out = UART(UART.UART1, 115200, 8, None, 1, timeout=1000, read_buf_len=4096)
sensor.reset(dual_buff=True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
#sensor.set_auto_exposure(False)
#sensor.set_contrast(3)#コントラスト
#sensor.set_brightness(2)#明るさ
#sensor.set_saturation(3)#彩
sensor.set_auto_gain(False)
#sensor.set_auto_whitebal(True)
sensor.set_auto_exposure(False, exposure_us=250)
sensor.set_auto_whitebal(False,(65, 61, 160))
sensor.run(1)
sensor.skip_frames(time = 2000)
#--------------------------------------------------------------------------------------
#variable------------------------------------------------------------------------------
time_stamp = time.ticks_ms();
startTime = 0
robo_center = (197,323)
catchxy = [223, 236, 181, 190]
yokoline=30#画面上での横線の座標
tateline=290#画面上での縦線の座標
DX = 0
DY = -25
POINT_180 = [
(72+DX,141+DY,10), (64+DX,114+DY,20), (60+DX,99+DY,30), (57+DX,90+DY,40), (56+DX,85+DY,50),
(55+DX,80+DY,60), (54+DX,77+DY,70), (53+DX,74+DY,80), (52+DX,72+DY,90), (52+DX,71+DY,100)
]
POINT_1575 = [
(108+DX,124+DY,10), (105+DX,94+DY,20), (104+DX,77+DY,30), (103+DX,67+DY,40), (103+DX,61+DY,50),
(102+DX,56+DY,60), (102+DX,52+DY,70), (102+DX,50+DY,80), (102+DX,48+DY,90), (102+DX,46+DY,100)
]
POINT_135 = [
(147+DX,118+DY,10), (151+DX,86+DY,20), (153+DX,70+DY,30), (154+DX,59+DY,40), (155+DX,52+DY,50),
(156+DX,47+DY,60), (156+DX,43+DY,70), (156+DX,41+DY,80), (156+DX,39+DY,90), (157+DX,37+DY,100)
]
POINT_1125 = [
(188+DX,122+DY,10), (198+DX,91+DY,20), (204+DX,73+DY,30), (207+DX,64+DY,40), (209+DX,57+DY,50),
(211+DX,53+DY,60), (212+DX,50+DY,70), (213+DX,47+DY,80), (213+DX,45+DY,90), (214+DX,43+DY,100)
]
POINT_90 = [
(227+DX,139+DY,10), (243+DX,110+DY,20), (252+DX,94+DY,30), (257+DX,85+DY,40), (261+DX,79+DY,50),
(263+DX,75+DY,60), (264+DX,72+DY,70), (266+DX,69+DY,80), (267+DX,67+DY,90), (268+DX,66+DY,100)
]
court_max = [0,0,0,0,0]#一番遠くの検知した緑の距離
rectO=cxO=cyO=radsO=pixel1=0
img = None
BX = 165 #ボール検知範囲円の中心のX座標
BY = 240 #ボール検知範囲円の中心のy座標
BR = 240 #ボール検知範囲円の半径
DESIGN = True
#--------------------------------------------------------------------------------------
#memo必ず目を通すこと-----------------------------------------------------------------------
"""
カメラライダーの検知位置が正しいかどうか確認すること（自分で距離を測れる治具を作ろう）DX,DYで位置調整可能
ボール検知範囲上限（水色の円）が適切か確認すること。BX,BY,BRで設定可能
緑色の閾値はLの最小値の設定だけ壁の色がとられない位置に調整するだけでOK
DESIGNの値をFalseに変えると、描画が実行されなくなる。ライダーがボール用の描画を誤検知する可能性があるので、試走時はFalse
"""
#--------------------------------------------------------------------------------------
#loop1---------------------------------------------------------------------------------
def check_green(points):
    distance_max = 0;
    pixel_sum = [0, 0, 0]
    ex_b = 0;
    ex_y = 0;
    pixel_color = []
    for x, y, d in points:
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                pixel = image.rgb_to_lab(img.get_pixel(x + dx, y + dy))
                pixel_sum[0] += pixel[0]
                pixel_sum[1] += pixel[1]
                pixel_sum[2] += pixel[2]
        pixel_sum[0] /= 9
        pixel_sum[1] /= 9
        pixel_sum[2] /= 9
        if thresholds_b[0] <= pixel[0] <= thresholds_b[1] and \
           thresholds_b[2] <= pixel[1] <= thresholds_b[3] and \
           thresholds_b[4] <= pixel[2] <= thresholds_b[5]:
            pixel_color.append('b')
            ex_b = 1;
        elif thresholds_y[0] <= pixel[0] <= thresholds_y[1] and \
             thresholds_y[2] <= pixel[1] <= thresholds_y[3] and \
             thresholds_y[4] <= pixel[2] <= thresholds_y[5]:
            pixel_color.append('y')
            ex_y = 2;
        elif thresholds_g[0] <= pixel[0] <= thresholds_g[1] and \
             thresholds_g[2] <= pixel[1] <= thresholds_g[3] and \
             thresholds_g[4] <= pixel[2] <= thresholds_g[5]:
            pixel_color.append('g')
            distance_max = d
        else:
            pixel_color.append('r')
    i = 0
    if DESIGN:
        for x, y, d in points:
            if pixel_color[i] == 'b':
                img.draw_circle(x,y,2,(0, 0, 255))
            elif pixel_color[i] == 'y':
                img.draw_circle(x,y,2,(255, 255, 0))
            elif pixel_color[i] == 'g':
                img.draw_circle(x,y,2,(0, 255, 0))
            elif pixel_color[i] == 'r':
                im.draw_circle(x,y,2,(255, 0, 0))
            i += 1;
    return distance_max + ex_b + ex_y

def loop1():
    global court_max
    while True:
        #cameraLiDAR===================================================================
        if not img == None:
            court_max[0] = check_green(POINT_180)#-180度
            court_max[1] = check_green(POINT_1575)#-157.5度
            court_max[2] = check_green(POINT_135)#-135度
            court_max[3] = check_green(POINT_1125)#-112.5度
            court_max[4] = check_green(POINT_90)#-90度
            time.sleep_ms(1)
        #==============================================================================
#--------------------------------------------------------------------------------------
#start_loop1---------------------------------------------------------------------------
# スレッド起動
_thread.start_new_thread(loop1, ())
#--------------------------------------------------------------------------------------
#loop0---------------------------------------------------------------------------------
while True:
    areaO=0#色の面積の最大値を配列にして保存
    val=0#画面中心を原点としたときのｘ座標
    vall=0#画面中心を原点としたときのｙ座標
    distance = -1
    drCatch = False
    try:
        img = sensor.snapshot()
    except:
        img = None;
    if not img == None:
        blobs = img.find_blobs([thresholds_o], pixels_threshold=1, area_threshold=4, merge=True,margin=3)#ボールの色探索
        if blobs:
            for b in blobs:
                if b.area() > areaO and ((b.cx() - BX) ** 2) + ((b.cy() - BY) ** 2) < BR ** 2:
                   areaO = b.area()
                   cxO=b.cx()
                   cyO=b.cy()
                   rectO=b.rect()
                   pixel1 = b.pixels()
                   val=b.cx()-197
                   vall=b.cy()-323
                   startTime = 0;
            radsO=int(math.degrees(math.atan2(-val,vall)))#角度
            if radsO < 0:
                radsO += 360
            radsO=(radsO - 150.6) * 90 / 45.2 + 90#角度補正式、、、信じろ！
            if radsO > 180:
                radsO = 180
            distance = int(math.sqrt((cyO-yokoline)**2+(cxO-tateline)**2))
        if areaO == 0:
            if startTime == 0:
                startTime = time.ticks_diff(time.ticks_ms(),time_stamp)
                pixel1 = 1
            elif 0 <= time.ticks_diff(time.ticks_ms(),time_stamp) - startTime <= 1000:
                pixel1 = 1
            else:
                rectO=cxO=cyO=radsO=pixel1=0

        if cxO!=0:#ボールの場所を画面に表示
          img.draw_rectangle(rectO,(255,0,0))#長方形の生成
          img.draw_cross(cxO, cyO) #中央のバツの生成
          if DESIGN:
              if areaO != 0:
                img.draw_line(197,323,cxO,cyO,(255,255,255))
              else:
                img.draw_line(197,323,cxO,cyO,(255,128,0))
          #img.draw_line(tateline,240,tateline-80,0,(0,255,255))
        else:
          radsO=185

        """
        areaB=0#色の面積の最大値を配列にして保存
        rectB=0#取得した中心のX座標、Y座標、ブロックの横幅、縦幅の値を配列に
        cxB=0#取得したx座標保存
        cyB=0#取得したY座標のみを保存
        radsB=0#ｒによって出た角度を絶対値に
        blobs = img.find_blobs([thresholds_b], pixels_threshold=1, area_threshold=3, merge=True,margin=3)
        if blobs:
            for b in blobs:
                if b.area() > areaB:
                   areaB = b.area()
                   cxB=b.cx()
                   cyB=b.cy()
                   rectB=b.rect()
            radsB=int(math.degrees(math.atan2(tateline - cxB,cyB - yokoline)))#角度
        if areaB!=0:#ゴールの場所を画面に表示
          img.draw_rectangle(rectB,(0,0,255))#長方形の生成
          img.draw_cross(cxB, cyB) #中央のバツの生成
        else:
            radsB=185
        """

        if uart_out.any():
            received_data = uart_out.read()
            if received_data.decode('utf-8') == "blb":
                drib = True
            elif received_data.decode('utf-8') == "ble":
                drib = False
        #img.draw_line(288,0,220,240)#180度
        #img.draw_line(222,0,203,240)#157.5度
        #img.draw_line(159,0,187,240)#135度
        #img.draw_line(92,0,170,240)#112.5度
        #img.draw_line(15,0,150,240)#90度
        if DESIGN:
            img.draw_line(0,yokoline,320,yokoline)#横線の描画
            img.draw_line(tateline,0,tateline,240)#縦線の描画
            img.draw_circle(BX,BY,BR,(0, 255, 255))
        pixeluart = int(pixel1*0.2)
        uart_out.write("r")
        uart_out.write(str(radsO))
        uart_out.write("s")
        uart_out.write(str(pixeluart))
        uart_out.write("l")
        uart_out.write(str(distance))
        uart_out.write("w")
        uart_out.write(str(int(0)))
        #uart_out.write("b")
        #uart_out.write(str(radsB))
        #コートの一番遠くの距離を送信
        uart_out.write("f")
        uart_out.write(str(court_max[0]))
        uart_out.write("g")
        uart_out.write(str(court_max[1]))
        uart_out.write("h")
        uart_out.write(str(court_max[2]))
        uart_out.write("i")
        uart_out.write(str(court_max[3]))
        uart_out.write("j")
        uart_out.write(str(court_max[4]))
        uart_out.write('e')
        print(startTime)
        time.sleep_ms(1);
#--------------------------------------------------------------------------------------

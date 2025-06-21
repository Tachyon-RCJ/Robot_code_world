import sensor, image, time, math#ライブラリーの読み込み
from pyb import UART, LED, Pin, Timer, DAC
import struct
green_led = LED(2)
#o:橙 g:緑 b:青 y:黄
thresholds1 = (3, 65, 21, 52, 29, 57)#ボールの色
#thresholds1 = [(44, 74, -36, -8, -25, 11)]
thresholds1a = (82, 100, -16, 8, -4, 19)#ボール捕捉エリア
thresholds_g = (32, 46, -29, -9, -3, 23)#コート色
thresholds_b = (16, 25, -7, -1, -16, -4)#青ゴール
thresholds_y = (49, 64, -32, -12, 49, 63)#黄色ゴール
thresholds4 = (84, 100, -11, 13, -25, 32)#白線
"""
thresholds1 = (56, 64, 33, 59, 40, 65)#ボールの色
#thresholds1 = [(44, 74, -36, -8, -25, 11)]
thresholds1a = (82, 100, -16, 8, -4, 19)#ボール捕捉エリア
thresholds_g = (32, 46, -29, -9, -3, 23)#コート色
thresholds_b = (36, 50, -10, 24, -52, -21)#青ゴール
thresholds_y = (65, 100, -27, -7, -22, 2)#黄色ゴール
thresholds4 = (84, 100, -11, 13, -25, 32)#白線
"""
uart = UART(3, 115200, timeout_char=100)
ren=0
renH = 0#ボール時々見えるときのrads保存
renH1 = 0#ボール時々見える時のriole保存
sensor.reset()
sensor.set_pixformat(sensor.RGB565)#カラースケール
sensor.set_framesize(sensor.QVGA)#解像度
sensor.skip_frames(time = 400)#描写速度
#sensor.set_contrast(2)#コントラスト
#sensor.set_brightness(-2)#明るさ
#sensor.set_saturation(1)#彩

sensor.set_auto_gain(False)
#sensor.set_auto_gain(False, gain_db=15)
sensor.set_auto_exposure(False, exposure_us=6500)#大会4500
"""

#sensor.set_auto_gain(False)
#sensor.set_auto_gain(False, gain_db=15)
#sensor.set_auto_exposure(False, exposure_us=4500)
"""
sensor.set_auto_whitebal(False, rgb_gain_db=(1.92073, 0.119987, 1.0006831))
clock = time.clock()
tim1 = Timer(4, freq=1000)
hoko=0 #真後ろぎりぎりにあるボールの回り込み方向保存
startTime = 0
shootMove = False
yokoline=170#画面上での横線の座標
tateline=153#画面上での縦線の座標 白162　黒152
point_675 = [
(tateline - 104,142,10), (tateline - 120,128,20), (tateline - 126,122,30), (tateline - 132,117,40), (tateline - 134,115,50),
(tateline - 136,113,60), (tateline - 137,112,70), (tateline - 139,111,80), (tateline - 140,110,90), (tateline - 141,109,100)
]
point_45 = [
(tateline - 74,117,10), (tateline - 85,99,20), (tateline - 92,89,30), (tateline - 95,83,40), (tateline - 97,80,50),
(tateline - 99,77,60), (tateline - 100,75,70), (tateline - 102,73,80), (tateline - 102,72,90), (tateline - 103,71,100)
]
point_225 = [
(tateline - 39,101,10), (tateline - 44,82,20), (tateline - 47,71,30), (tateline - 49,64,40), (tateline - 50,60,50),
(tateline - 52,56,60), (tateline - 52,53,70), (tateline - 53,52,80), (tateline - 53,51,90), (tateline - 54,49,100)
]
point0 = [
(tateline,97,10), (tateline,77,20), (tateline,65,30), (tateline,58,40), (tateline,54,50),
(tateline,50,60), (tateline,47,70), (tateline,45,80), (tateline,43,90), (tateline,42,100)
]
point225 = [
(tateline + 39,101,10), (tateline + 44,82,20), (tateline + 47,71,30), (tateline + 49,64,40), (tateline + 50,60,50),
(tateline + 52,56,60), (tateline + 52,53,70), (tateline + 53,52,80), (tateline + 53,51,90), (tateline + 54,49,100)
]
point45 = [
(tateline + 74,117,10), (tateline + 85,99,20), (tateline + 92,89,30), (tateline + 95,83,40), (tateline + 97,80,50),
(tateline + 99,77,60), (tateline + 100,75,70), (tateline + 102,73,80), (tateline + 102,72,90), (tateline + 103,71,100)
]
point675 = [
(tateline + 104,142,10), (tateline + 120,128,20), (tateline + 126,122,30), (tateline + 132,117,40), (tateline + 134,115,50),
(tateline + 136,113,60), (tateline + 137,112,70), (tateline + 139,111,80), (tateline + 140,110,90), (tateline + 141,109,100)
]
court_max = [0,0,0,0,0,0,0]#一番遠くの検知した緑の距離

#周囲検知関数
def check_green(points):
    distance_max = 0;
    pixel_sum = [0, 0, 0]
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
        if thresholds_g[0] <= pixel[0] <= thresholds_g[1] and \
           thresholds_g[2] <= pixel[1] <= thresholds_g[3] and \
           thresholds_g[4] <= pixel[2] <= thresholds_g[5]:
            img.draw_circle(x,y,2,(0, 255, 0))
            distance_max = max(distance_max,d)
        else:
            img.draw_circle(x,y,2,(255, 0, 0))
    return distance_max

while(True):
    kick_area = [tateline-17,yokoline-61,34,10]#[x,y,w,h]
    areaA=areaB=areaC=areaD=areaLine=0#色の面積の最大値を配列にして保存
    rectA=rectB=rectC=rectD=rectLine=0#取得した中心のX座標、Y座標、ブロックの横幅、縦幅の値を配列に
    cxA=cxB=cxC=cxD=0#取得したx座標保存
    cyA=cyB=cyC=cyD=0#取得したY座標のみを保存
    valk=val=val2=valc=vald=0#画面中心を原点としたときのｘ座標
    vallk=vall=vall2=vallc=valld=0#画面中心を原点としたときのｙ座標
    radsk=rads=rads2=rads3=rads4=0#ｒによって出た角度を絶対値に
    pixel1 = 0;
    kick = 0;
    distance = 115
    distance_b = 0
    distance_y = 0
    clock.tick()
    try:
        img = sensor.snapshot()
    except:
        img = None;
    if not img == None:

        blobs = img.find_blobs([thresholds1], pixels_threshold=1, area_threshold=1, merge=True,margin=3)#ボールの色探索
        if blobs:
            for b in blobs:
                if b.area() > areaA and not(tateline-65 < b.cx() < tateline+65 and yokoline-30 < b.cy() < yokoline+80) and math.sqrt((b.cx()-tateline)**2+(b.cy()-yokoline)**2) < 140:#yokoline:190半径:170
                    areaA = b.area()
                    cxA=b.cx()
                    cyA=b.cy()
                    rectA=b.rect()
                    pixel1 = b.pixels()
                    val=b.cx()-tateline
                    vall=b.cy()-yokoline
            distance = int(math.sqrt((cxA-tateline)**2+(cyA-yokoline)**2))
            rads=int(math.degrees(math.atan2(val,-vall))*45/32.1)#角度
        """
        blobsa = img.find_blobs([thresholds1a], pixels_threshold=1, area_threshold=1, merge=True,margin=3)#ボールの色探索
        if blobsa:
         for b in blobs:
             if b.area() > areaA and 129 < b.cx() < 172 and 94 < b.cy() < 104:
                areaA = b.area()
                cxA=b.cx()
                cyA=b.cy()
                rectA=b.rect()
                val=b.cx()-152
                vall=b.cy()-235
         rads=int(math.degrees(math.atan2(val,-vall))*45/32.1)#角度
         #print(pixel1)

         #if abs(rads) < 20:
            #rads = 0
         #else:
            #rads = rads*1.5
        """

        blobs = img.find_blobs([thresholds_b], pixels_threshold=1, area_threshold=100, merge=True,margin=3)#ボールの色探索
        if blobs:
         for b in blobs:
             if b.area() > areaC and not(tateline-75 < b.cx() < tateline+70 and 125 < b.cy() < yokoline+80):
                areaC = b.area()
                cxC=b.cx()
                cyC=b.cy()
                rectC=b.rect()
                valc=b.cx()-tateline
                vallc=b.cy()-yokoline
         rads3=int(math.degrees(math.atan2(valc,-vallc)))#角度
         #if rads3 >= 0:
            #rads3=math.floor(rads3/20)*20
         #else:
            #rads3=math.floor(abs(rads3)/20)*(-20)
         distance_b = int(math.sqrt((cxC-tateline)**2+(cyC-yokoline)**2))


         blobs = img.find_blobs([thresholds_y], pixels_threshold=1, area_threshold=100, merge=True,margin=3)#ボールの色探索
        if blobs:
         for b in blobs:
             if b.area() > areaD and not(tateline-75 < b.cx() < tateline+70 and 125 < b.cy() < yokoline+80):
                areaD = b.area()
                cxD=b.cx()
                cyD=b.cy()
                rectD=b.rect()
                vald=b.cx()-tateline
                valld=b.cy()-yokoline
         rads4=int(math.degrees(math.atan2(vald,-valld)))#角度
         #if rads4 >= 0:
            #rads4=math.floor(rads4/20)*20
        # else:
            #rads4=math.floor(abs(rads4)/20)*(-20)
         distance_y = int(math.sqrt((cxD-tateline)**2+(cyD-yokoline)**2))


        """
        blobs = img.find_blobs([thresholds4[threshold_index]], pixels_threshold=100, area_threshold=1, merge=True,margin=3)
        if blobs:
        for b in blobs:
            if b.area() > areaLine:
                areaLine = b.area()
                rectLine = b.rect()
        whiteLine = img.find_lines(roi=rectLine,threshold=1300, theta_margin=25, rho_margin=25)#ボールの色探索
        if whiteLine:
            img.draw_line(whiteLine[0].line(), color = (255, 0, 0))
        """
        """
        if uart.any() > 0:
        received = uart.read().decode()
        if (len(received) > 0):
            if received == "bls":
                LED(1).on()
                uart.write("c")
                startTime = time.ticks_ms()
                shootMove = True
        """

        if areaA!=0:#ボールの場所を画面に表示
            img.draw_rectangle(rectA,(255,0,0))#長方形の生成
            img.draw_cross(cxA, cyA) #中央のバツの生成
            #img.draw_line(tateline,yokoline,cxA,cyA,(255,0,0))
            #img.draw_cross(cxC, cyC) #中央のバツの生成
            #img.draw_arrow(tateline,yokoline,tateline+int(math.sqrt(val**2+vall**2)*math.sin(math.radians(rads))),yokoline-int(math.sqrt(val**2+vall**2)*math.cos(math.radians(rads))),(150,0,150))
            green_led.on()
            if kick_area[0] <= cxA <= kick_area[0]+kick_area[2] and \
                kick_area[1] <= cyA <= kick_area[1]+kick_area[3]:
                kick = 1

        else:
            rads=185
            green_led.off()

        #if areaLine != 0:
        #img.draw_rectangle(rectLine,(255,255,255))#長方形の生成

        if areaC!=0:#ゴールの場所を画面に表示
            img.draw_rectangle(rectC,(0,0,255))#長方形の生成
        else:
            rads3=185
        if areaD!=0:#ゴールの場所を画面に表示
            img.draw_rectangle(rectD,(255,255,0))#長方形の生成
        else:
            rads4=185

        #if distance_b > 200:
            #rads3 = 185
        if distance_y > 200:
            rads4 = 185
        rads4 = 185

        court_max[0] = check_green(point675)#67.5度
        court_max[1] = check_green(point45)#45度
        court_max[2] = check_green(point225)#22.5度
        court_max[3] = check_green(point0)#0度
        court_max[4] = check_green(point_225)#-22.5度
        court_max[5] = check_green(point_45)#-45度
        court_max[6] = check_green(point_675)#-67.5度

        preRads = rads
        img.draw_rectangle(tateline-65,yokoline-30,130,100,(0,255,255))
        img.draw_line(0,yokoline,320,yokoline)#横線の描画
        img.draw_line(tateline,0,tateline,240)#縦線の描画
        img.draw_line(152,235,cxA,cyA)
        img.draw_rectangle(kick_area,(255,0,255))
        radsuart1 = int((rads + 180)*0.694)
        radsuart3 = int((rads3 + 180)*0.694)
        radsuart4 = int((rads4 + 180)*0.694)
        pixeluart = int(pixel1*0.3)
        if distance >= 110:
            distance = 110
        uart.write("f")
        uart.writechar(radsuart1)
        uart.write("b")
        uart.writechar(radsuart3)
        uart.write("y")
        uart.writechar(radsuart4)
        uart.write("l")
        uart.writechar(distance)
        uart.write("v")
        uart.writechar(pixeluart)
        uart.write("a")
        uart.writechar(kick)
        #コートの一番遠くの距離を送信
        uart.write("o")
        uart.writechar(court_max[0])
        uart.write("p")
        uart.writechar(court_max[1])
        uart.write("q")
        uart.writechar(court_max[2])
        uart.write("r")
        uart.writechar(court_max[3])
        uart.write("s")
        uart.writechar(court_max[4])
        uart.write("t")
        uart.writechar(court_max[5])
        uart.write("u")
        uart.writechar(court_max[6])

        print(court_max)
        #print(int(math.degrees(math.atan2(val,-vall))),rads, math.sqrt(val**2+vall**2))

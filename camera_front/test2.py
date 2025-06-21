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
sensor.set_auto_whitebal(False,(1.92073, 0.119987, 1.0006831))
clock = time.clock()
tim1 = Timer(4, freq=1000)
hoko=0 #真後ろぎりぎりにあるボールの回り込み方向保存
startTime = 0
shootMove = False
#周囲検知関数
def check_green_way0(img,x1,y1,x2,y2,machine_area,hugo):
    distance_max = 0;
    for i1 in range(20):
        i = int(i1 * 235/20)
        if i != 0:
            x = int(x1 + (x2 - x1) * (i - 1) / (y1 - y2))
            y = 235 - (i - 1)
            pixel_o = image.rgb_to_lab(img.get_pixel(x, y))
        else:
            pixel_o = img.get_pixel(0, 0)
        if i != 234:
            x = int(x1 + (x2 - x1) * (i + 1) / (y1 - y2))
            y = 235 - (i + 1)
            pixel_n = image.rgb_to_lab(img.get_pixel(x, y))
        else:
            pixel_n = img.get_pixel(0, 0)
        x = int(x1 + (x2 - x1) * i / (y1 - y2))
        y = 235 - i
        pixel = image.rgb_to_lab(img.get_pixel(x, y))
        if i < machine_area:
            img.draw_circle(x,y,1,(255, 0, 0))
        else:
            if thresholds_g[0] <= pixel[0] <= thresholds_g[1] and \
               thresholds_g[2] <= pixel[1] <= thresholds_g[3] and \
               thresholds_g[4] <= pixel[2] <= thresholds_g[5]:
                if(thresholds_g[0] <= pixel_o[0] <= thresholds_g[1] and \
                   thresholds_g[2] <= pixel_o[1] <= thresholds_g[3] and \
                   thresholds_g[4] <= pixel_o[2] <= thresholds_g[5]) or \
                  (thresholds_g[0] <= pixel_n[0] <= thresholds_g[1] and \
                   thresholds_g[2] <= pixel_n[1] <= thresholds_g[3] and \
                   thresholds_g[4] <= pixel_n[2] <= thresholds_g[5]):
                    img.draw_circle(x,y,1,(0, 255, 0))
                    if distance_max < int(math.sqrt((x - int(x1 + (x2 - x1) * machine_area / 240))**2 + (y - machine_area)**2)):
                        distance_max = int(math.sqrt((x - int(x1 + (x2 - x1) * machine_area / 240))**2 + (y - machine_area)**2))
            else:
                if(thresholds_g[0] <= pixel_o[0] <= thresholds_g[1] and \
                   thresholds_g[2] <= pixel_o[1] <= thresholds_g[3] and \
                   thresholds_g[4] <= pixel_o[2] <= thresholds_g[5]) and \
                  (thresholds_g[0] <= pixel_n[0] <= thresholds_g[1] and \
                   thresholds_g[2] <= pixel_n[1] <= thresholds_g[3] and \
                   thresholds_g[4] <= pixel_n[2] <= thresholds_g[5]):
                    img.draw_circle(x,y,1,(0, 255, 0))
            """
            if thresholds_b[0] <= pixel[0] <= thresholds_b[1] and \
               thresholds_b[2] <= pixel[1] <= thresholds_b[3] and \
               thresholds_b[4] <= pixel[2] <= thresholds_b[5]:
                if(thresholds_b[0] <= pixel_o[0] <= thresholds_b[1] and \
                   thresholds_b[2] <= pixel_o[1] <= thresholds_b[3] and \
                   thresholds_b[4] <= pixel_o[2] <= thresholds_b[5]) or \
                  (thresholds_b[0] <= pixel_n[0] <= thresholds_b[1] and \
                   thresholds_b[2] <= pixel_n[1] <= thresholds_b[3] and \
                   thresholds_b[4] <= pixel_n[2] <= thresholds_b[5]):
                    img.draw_circle(x,y,1,(0, 255, 0))
            else:
                if(thresholds_b[0] <= pixel_o[0] <= thresholds_b[1] and \
                   thresholds_b[2] <= pixel_o[1] <= thresholds_b[3] and \
                   thresholds_b[4] <= pixel_o[2] <= thresholds_b[5]) and \
                  (thresholds_b[0] <= pixel_n[0] <= thresholds_b[1] and \
                   thresholds_b[2] <= pixel_n[1] <= thresholds_b[3] and \
                   thresholds_b[4] <= pixel_n[2] <= thresholds_b[5]):
                    img.draw_circle(x,y,1,(0, 255, 0))
            if thresholds_y[0] <= pixel[0] <= thresholds_y[1] and \
               thresholds_y[2] <= pixel[1] <= thresholds_y[3] and \
               thresholds_y[4] <= pixel[2] <= thresholds_y[5]:
                if(thresholds_y[0] <= pixel_o[0] <= thresholds_y[1] and \
                   thresholds_y[2] <= pixel_o[1] <= thresholds_y[3] and \
                   thresholds_y[4] <= pixel_o[2] <= thresholds_y[5]) or \
                  (thresholds_y[0] <= pixel_n[0] <= thresholds_y[1] and \
                   thresholds_y[2] <= pixel_n[1] <= thresholds_y[3] and \
                   thresholds_y[4] <= pixel_n[2] <= thresholds_y[5]):
                    img.draw_circle(x,y,1,(0, 255, 0))
            else:
                if(thresholds_g[0] <= pixel_o[0] <= thresholds_y[1] and \
                   thresholds_g[2] <= pixel_o[1] <= thresholds_y[3] and \
                   thresholds_g[4] <= pixel_o[2] <= thresholds_y[5]) and \
                  (thresholds_g[0] <= pixel_n[0] <= thresholds_y[1] and \
                   thresholds_g[2] <= pixel_n[1] <= thresholds_y[3] and \
                   thresholds_g[4] <= pixel_n[2] <= thresholds_y[5]):
                    img.draw_circle(x,y,1,(0, 255, 0))
            """
    return distance_max

def check_green_way1(img,x1,y1,x2,y2,machine_area,hugo):
    distance_max = 0;
    for i1 in range(20):
        i = int(i1 * 152/20)
        if i != 0:
            x = 152 + (i - 1) * hugo
            y = int(y1 + (y2 - y1) * (i - 1) / 152)
            pixel_o = image.rgb_to_lab(img.get_pixel(x, y))
        else:
            pixel_o = img.get_pixel(0, 0)
        if i != 151:
            x = 152 + (i - 1) * hugo
            y = int(y1 + (y2 - y1) * (i + 1) / 152)
            pixel_n = image.rgb_to_lab(img.get_pixel(x, y))
        else:
            pixel_n = img.get_pixel(0, 0)
        x = 152 + i * hugo
        y = int(y1 + (y2 - y1) * i / 152)
        pixel = image.rgb_to_lab(img.get_pixel(x, y))
        if i < machine_area:
            img.draw_circle(x,y,1,(255, 0, 0))
        else:
            if thresholds_g[0] <= pixel[0] <= thresholds_g[1] and \
               thresholds_g[2] <= pixel[1] <= thresholds_g[3] and \
               thresholds_g[4] <= pixel[2] <= thresholds_g[5]:
                if(thresholds_g[0] <= pixel_o[0] <= thresholds_g[1] and \
                   thresholds_g[2] <= pixel_o[1] <= thresholds_g[3] and \
                   thresholds_g[4] <= pixel_o[2] <= thresholds_g[5]) or \
                  (thresholds_g[0] <= pixel_n[0] <= thresholds_g[1] and \
                   thresholds_g[2] <= pixel_n[1] <= thresholds_g[3] and \
                   thresholds_g[4] <= pixel_n[2] <= thresholds_g[5]):
                    img.draw_circle(x,y,1,(0, 255, 0))
                    if distance_max < int(math.sqrt((x - (152 + machine_area * hugo))**2 + (y - int(y1 + (y2 - y1) * machine_area / 152))**2)):
                        distance_max = int(math.sqrt((x - (152 + machine_area * hugo))**2 + (y - int(y1 + (y2 - y1) * machine_area / 152))**2))
            else:
                if(thresholds_g[0] <= pixel_o[0] <= thresholds_g[1] and \
                   thresholds_g[2] <= pixel_o[1] <= thresholds_g[3] and \
                   thresholds_g[4] <= pixel_o[2] <= thresholds_g[5]) and \
                  (thresholds_g[0] <= pixel_n[0] <= thresholds_g[1] and \
                   thresholds_g[2] <= pixel_n[1] <= thresholds_g[3] and \
                   thresholds_g[4] <= pixel_n[2] <= thresholds_g[5]):
                    img.draw_circle(x,y,1,(0, 255, 0))
                    if distance_max < int(math.sqrt((x - (152 + machine_area * hugo))**2 + (y - int(y1 + (y2 - y1) * machine_area / 152))**2)):
                        distance_max = int(math.sqrt((x - (152 + machine_area * hugo))**2 + (y - int(y1 + (y2 - y1) * machine_area / 152))**2))
            """
            if thresholds_b[0] <= pixel[0] <= thresholds_b[1] and \
               thresholds_b[2] <= pixel[1] <= thresholds_b[3] and \
               thresholds_b[4] <= pixel[2] <= thresholds_b[5]:
                if(thresholds_b[0] <= pixel_o[0] <= thresholds_b[1] and \
                   thresholds_b[2] <= pixel_o[1] <= thresholds_b[3] and \
                   thresholds_b[4] <= pixel_o[2] <= thresholds_b[5]) or \
                  (thresholds_b[0] <= pixel_n[0] <= thresholds_b[1] and \
                   thresholds_b[2] <= pixel_n[1] <= thresholds_b[3] and \
                   thresholds_b[4] <= pixel_n[2] <= thresholds_b[5]):
                    img.draw_circle(x,y,1,(0, 255, 0))
            else:
                if(thresholds_b[0] <= pixel_o[0] <= thresholds_b[1] and \
                   thresholds_b[2] <= pixel_o[1] <= thresholds_b[3] and \
                   thresholds_b[4] <= pixel_o[2] <= thresholds_b[5]) and \
                  (thresholds_b[0] <= pixel_n[0] <= thresholds_b[1] and \
                   thresholds_b[2] <= pixel_n[1] <= thresholds_b[3] and \
                   thresholds_b[4] <= pixel_n[2] <= thresholds_b[5]):
                    img.draw_circle(x,y,1,(0, 255, 0))
            if thresholds_y[0] <= pixel[0] <= thresholds_y[1] and \
               thresholds_y[2] <= pixel[1] <= thresholds_y[3] and \
               thresholds_y[4] <= pixel[2] <= thresholds_y[5]:
                if(thresholds_y[0] <= pixel_o[0] <= thresholds_y[1] and \
                   thresholds_y[2] <= pixel_o[1] <= thresholds_y[3] and \
                   thresholds_y[4] <= pixel_o[2] <= thresholds_y[5]) or \
                  (thresholds_y[0] <= pixel_n[0] <= thresholds_y[1] and \
                   thresholds_y[2] <= pixel_n[1] <= thresholds_y[3] and \
                   thresholds_y[4] <= pixel_n[2] <= thresholds_y[5]):
                    img.draw_circle(x,y,1,(0, 255, 0))
            else:
                if(thresholds_g[0] <= pixel_o[0] <= thresholds_y[1] and \
                   thresholds_g[2] <= pixel_o[1] <= thresholds_y[3] and \
                   thresholds_g[4] <= pixel_o[2] <= thresholds_y[5]) and \
                  (thresholds_g[0] <= pixel_n[0] <= thresholds_y[1] and \
                   thresholds_g[2] <= pixel_n[1] <= thresholds_y[3] and \
                   thresholds_g[4] <= pixel_n[2] <= thresholds_y[5]):
                    img.draw_circle(x,y,1,(0, 255, 0))
            """
    return distance_max

while(True):
    yokoline=170#画面上での横線の座標
    tateline=153#画面上での縦線の座標 白162　黒152
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
    court_max = [0,0,0,0,0,0,0]#一番遠くの検知した緑の距離
    try:
        img = sensor.snapshot()
    except:
        img = None;
    if not img == None:

        #周囲検知
        court_max[0] = check_green_way1(img,152,235,304,99,100,1)#67.5度
        court_max[1] = check_green_way0(img,152,235,304,-7,105,1)#45度
        court_max[2] = check_green_way0(img,152,235,304,-292,110,1)#22.5度
        court_max[3] = check_green_way0(img,152,235,152,0,115,1)#0度
        court_max[4] = check_green_way0(img,152,235,0,-292,110,-1)#-22.5度
        court_max[5] = check_green_way0(img,152,235,0,-7,105,-1)#-45度
        court_max[6] = check_green_way1(img,152,235,0,99,100,-1)#-67.5度

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
         #distance_y = int(math.sqrt((cxD-tateline)**2+(cyD-yokoline)**2))


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

        print(kick)
        #print(int(math.degrees(math.atan2(val,-vall))),rads, math.sqrt(val**2+vall**2))

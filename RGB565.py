#黑色灰度线以下示例
#在机器人之后做一条生产线需要付出很多努力。
#这个示例脚本显示如何做机器人的机器视觉部分。
#您可以使用此脚本的输出来驱动差速驱动器机器人一行。
#这个脚本只是产生一个单一的转向值来告诉你的机器人向左或向右移动
#为了使脚本正常工作，您应该将相机指向一行45度角度。
#请确保只有线路在内相机的视野。

import sensor, image, time, math, pyb
from pyb import UART

#下面是定义一个去控制鱼速度和角度的函数speed是速度值deg是角度值
def PackageSend():
   uart = UART(3, 9600)
   speedflag=0xD0|(0x0F&speed)
   speedsend = bytes([170,145,speedflag,252])
   uart.write(speedsend)
   pyb.delay(50)
   degflag=0xE0|(0x0F&deg);
   degsend = bytes([170,145,degflag,252])
   uart.write(degsend)
   pyb.delay(50)
   print(speed,deg)
# 跟踪一条黑线。 使用[（128，255）]来追踪白线。
GRAYSCALE_THRESHOLD = [(    44,   100,  -5,   9,  -11,   10)]
black_threshold   = [(    1,    2,    -2,  2,    -1,    2)]
#设置阈值，如果是黑线，GRAYSCALE_THRESHOLD = [(0, 64)]；如果是白线，GRAYSCALE_THRESHOLD = [(128,255)]


# 每个roi是（x，y，w，h）。 线检测算法将尝试找到
# 每个roi中最大斑点的质心。 重心的x位置
# 然后将以分配最多权重的不同权重进行平均
# 到靠近图像底部的roi，而不是下一个roi等等。
ROIS = [ # [ROI, weight]
        (0, 100, 160, 20, 0.7), # 你需要调整你的应用程序的权重。
        (0, 050, 160, 20, 0.3), # 取决于您的机器人的设置。
        (0, 000, 160, 20, 0.1)
       ]
#roi代表三个取样区域，（x,y,w,h,weight）,代表左上顶点（x,y）宽高分别为w和h的矩形，
#weight为当前矩形的权值。注意本例程采用的QQVGA图像大小为160x120，roi即把图像横分成三个矩形。
#三个矩形的阈值要根据实际情况进行调整，离机器人视野最近的矩形权值要最大，
#如上图的最下方的矩形，即(0, 100, 160, 20, 0.7)

# 计算权重除数（我们计算这个，所以你不必使权重添加到1）。
weight_sum = 0 #权值和初始化
for r in ROIS: weight_sum += r[4] # r[4] is the roi weight.
#计算权值和。遍历上面的三个矩形，r[4]即每个矩形的权值。

# 相机初始化...
sensor.reset() # 初始化相机传感器。
sensor.set_pixformat(sensor.RGB565) # 使用颜色图。
sensor.set_framesize(sensor.QQVGA) # 使用QQVGA速度。
sensor.skip_frames(30) # 让新设置生效。
sensor.set_auto_gain(False) # 必须关闭颜色跟踪
sensor.set_auto_whitebal(False) #必须关闭颜色跟踪
#关闭白平衡

clock = time.clock() # 跟踪FPS。
led = pyb.LED(4) # 切换到使用绿色LED。

while(True):
    clock.tick() # 跟踪快照（）之间的毫秒数。
    img = sensor.snapshot() # 拍照并返回图像。

    blobs2 = img.find_blobs(black_threshold, roi=r[0:4], merge=True)
#find_blobs(thresholds, invert=False, roi=Auto),thresholds为颜色阈值，
#是一个元组，需要用括号［ ］括起来。invert=1,反转颜色阈值，invert=False默认
#不反转。roi设置颜色识别的视野区域，roi是一个元组， roi = (x, y, w, h)，代表
#从左上顶点(x,y)开始的宽为w高为h的矩形区域，roi不设置的话默认为整个图像视野。
#这个函数返回一个列表，[0]代表识别到的目标颜色区域左上顶点的x坐标，［1］代表
#左上顶点y坐标，［2］代表目标区域的宽，［3］代表目标区域的高，［4］代表目标
#区域像素点的个数，［5］代表目标区域的中心点x坐标，［6］代表目标区域中心点y坐标，
#［7］代表目标颜色区域的旋转角度（是弧度值，浮点型，列表其他元素是整型），
#［8］代表与此目标区域交叉的目标个数，［9］代表颜色的编号（它可以用来分辨这个
#区域是用哪个颜色阈值threshold识别出来的）。
    if blobs2:      #如果找到了目标颜色
     pyb.delay(800)
     if blobs2:
      led.on()             #亮灯
      speed=15
      deg=7
      PackageSend()
      time.sleep(2000)     #延时2s
    else: led.off()
    centroid_sum = 0
    #利用颜色识别分别寻找三个矩形区域内的线段
    for r in ROIS:
        blobs1 = img.find_blobs(GRAYSCALE_THRESHOLD, roi=r[0:4], merge=True) # r[0:4] is roi tuple.
        #找到视野中的线,merge=true,将找到的图像区域合并成一个

        #目标区域找到直线
        if blobs1:
            # Find the index of the blob with the most pixels.
            most_pixels = 0
            largest_blob = 0
            for i in range(len(blobs1)):
            #目标区域找到的颜色块（线段块）可能不止一个，找到最大的一个，作为本区域内的目标直线
                if blobs1[i].pixels() > most_pixels:
                    most_pixels = blobs1[i].pixels()
                    #merged_blobs[i][4]是这个颜色块的像素总数，如果此颜色块像素总数大于
                    #most_pixels，则把本区域作为像素总数最大的颜色块。更新most_pixels和largest_blob
                    largest_blob = i

        # Draw a rect around the blob.
            img.draw_rectangle(blobs1[largest_blob].rect())
        #将此区域的像素数最大的颜色块画矩形和十字形标记出来
            img.draw_cross(blobs1[largest_blob].cx(),
                           blobs1[largest_blob].cy())

            centroid_sum += blobs1[largest_blob].cx() * r[4] # r[4] is the roi weight.
        #计算centroid_sum，centroid_sum等于每个区域的最大颜色块的中心点的x坐标值乘本区域的权值

    center_pos = (centroid_sum / weight_sum) # 确定线的中心。
#中间公式

# 将center_pos转换为偏转角。
# 我们使用非线性操作使得响应越来越强才能使得我们越走越远。
# 非线性操作对算法的输出使用是很好的。
# 像这样引起响应“触发”。
    deflection_angle = 0
#机器人应该转的角度

# 80是从X res的一半，60是从Y res的一半。
# 下面的等式只是计算三角形的角度。
# 其中三角形的相对侧是中心位置与中心的偏差，相邻侧是Y res的一半。
# 这个限制角度输出在-45°到45°之间。
    deflection_angle = -math.atan((center_pos-80)/60)
#角度计算.80 60 分别为图像宽和高的一半，图像大小为QQVGA 160x120.
#注意计算得到的是弧度值

# Convert angle in radians to degrees.
    deflection_angle = math.degrees(deflection_angle)
    deflection_angle = int(deflection_angle)
#将计算结果的弧度值转化为角度值

#现在你有一个角度告诉你多少转动机器人
#结合最靠近机器人的部分线和部分
#更远离机器人的线更好的预测。
#将结果打印在terminal中

    print(clock.fps()) # Note: Your OpenMV Cam runs about half as fast while
    q=abs(deflection_angle)#对数据加绝对值
    #判断如果角度大于-10°小于10°则小鱼以15的速度走直线
    if q<10:
     print( "angle: %f" % deflection_angle)
     speed=15
     deg=7
     PackageSend()
    if 10<=deflection_angle<=40:#判断如果角度大于10°小于40°则鱼头以13的速度向左拐
     print( "angle: %f" % deflection_angle)
     speed=13
     deg=11
     PackageSend()
    if -40<=deflection_angle<=-10:#判断如果角度大于-40°小于-10°则鱼头以13的速度向右拐
     print( "angle: %f" % deflection_angle)
     speed=13
     deg=4
     PackageSend()
    if deflection_angle<-40:#报错
     print( "abgle: %f" % deflection_angle)
     speed=13
     deg=0
     PackageSend()
    if deflection_angle>40:#遇到转弯前面全是水的区域了
     print( "angle: %f" % deflection_angle)
     speed=13
     deg=0
     PackageSend()
# 连接到您的计算机。 一旦断开连接，FPS应该增加。

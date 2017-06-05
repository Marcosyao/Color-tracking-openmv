# Untitled - By: MBENBEN - 周三 5月 24 2017

# Line Following Example
#
# Making a line following robot requires a lot of effort. This example script
# shows how to do the computer vision part of the line following robot. You
# can use the output from this script to drive a differential drive robot to
# follow a line. This script just generates a single turn value that tells
# your robot to go left or right.
#
# For this script to work properly you should point the camera at a line at a
# 45 or so degree angle. Please make sure that only the line is within the
# camera's field of view.
import sensor, image, time, math, pyb
from pyb import UART
from pyb import Pin
pin0 =Pin('P0',Pin.OUT_PP,Pin.PULL_UP)
#------------------------------------------------------------------------------------------------
#下面是定义一个去控制鱼速度和角度的函数speed是速度值deg是角度值
#uart sending
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
#------------------------------------------------------------------------------------------------
# Tracks a white line. Use [(0, 64)] for a tracking a black line.

GRAYSCALE_THRESHOLD = [(128, 255)]         #White line threshold

#设置阈值，如果是黑线，GRAYSCALE_THRESHOLD = [(0, 64)]；如果是白线，GRAYSCALE_THRESHOLD = [(128，255)]
#------------------------------------------------------------------------------------------------
# Each roi is (x, y, w, h). The line detection algorithm will try to find the
# centroid of the largest blob in each roi. The x position of the centroids
# will then be averaged with different weights where the most weight is assigned
# to the roi near the bottom of the image and less to the next roi and so on.
ROIS = [ # [ROI, weight]
        (0, 100, 160, 20, 0.7), # You'll need to tweak the weights for you app
        (0, 050, 160, 20, 0.3), # depending on how your robot is setup.
        (0, 000, 160, 20, 0.1)
       ]
#roi代表三个取样区域，（x,y,w,h,weight）,代表左上顶点（x,y）宽高分别为w和h的矩形，
#weight为当前矩形的权值。注意本例程采用的QQVGA图像大小为160x120，roi即把图像横分成三个矩形。
#三个矩形的阈值要根据实际情况进行调整，离机器人视野最近的矩形权值要最大，
#如上图的最下方的矩形，即(0, 100, 160, 20, 0.7)

#------------------------------------------------------------------------------------------------
# Compute the weight divisor
weight_sum = 0 #权值和初始化
for r in ROIS: weight_sum += r[4]
#计算权值和。遍历上面的三个矩形，r[4]即每个矩形的权值。
#------------------------------------------------------------------------------------------------
# Camera setup...
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # use grayscale.
sensor.set_framesize(sensor.QQVGA) # use QQVGA for speed.
sensor.skip_frames(10) # Let new settings take affect.
sensor.set_auto_gain(False) # 必须关闭颜色跟踪
sensor.set_auto_whitebal(False) #必须关闭颜色跟踪
#关闭白平衡
clock = time.clock() # Tracks FPS.
#------------------------------------------------------------------------------------------------
while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.

    centroid_sum = 0
    #利用颜色识别分别寻找三个矩形区域内的线段
#------------------------------------------------------------------------------------------------
#find the black area and send it to IO
    for blob in img.find_blobs([(0,25)], pixels_threshold=100, area_threshold=100, merge=True):
        img.draw_rectangle(blob.rect())
        img.draw_cross(blob.cx(), blob.cy())
        pin0.value(True)
        time.sleep(10)
        pin0.value(False)
#------------------------------------------------------------------------------------------------
    for r in ROIS:
        blobs = img.find_blobs(GRAYSCALE_THRESHOLD, roi=r[0:4],merge=True) # r[0:4] is roi tuple.
        #找到视野中的线
        #merged_blobs = img.find_markers(blobs) # merge overlapping blobs
        #将找到的图像区域合并成一个。

        #目标区域找到直线
        if blobs:
            # Find the index of the blob with the most pixels.
            most_pixels = 0
            largest_blob = 0
            for i in range(len(blobs)):
            #目标区域找到的颜色块（线段块）可能不止一个，找到最大的一个，作为本区域内的目标直线
                if blobs[i][4] > most_pixels:
                    most_pixels = blobs[i][4] # [4] is pixels.
                    #merged_blobs[i][4]是这个颜色块的像素总数，如果此颜色块像素总数大于                     #most_pixels，则把本区域作为像素总数最大的颜色块。更新most_pixels和largest_blob
                    largest_blob = i

            # Draw a rect around the blob.
            #将此区域的像素数最大的颜色块画矩形和十字形标记出来
            img.draw_rectangle(blobs[largest_blob][0:4]) # rect
            img.draw_cross(blobs[largest_blob][5], # cx
                           blobs[largest_blob][6]) # cy

            # [5] of the blob is the x centroid - r[4] is the weight.
            centroid_sum += blobs[largest_blob][5] * r[4]
            #计算centroid_sum，centroid_sum等于每个区域的最大颜色块的中心点的x坐标值乘本区域的权值

    center_pos = (centroid_sum / weight_sum) # Determine center of line.
    #中间公式

    # Convert the center_pos to a deflection angle. We're using a non-linear
    # operation so that the response gets stronger the farther off the line we
    # are. Non-linear operations are good to use on the output of algorithms
    # like this to cause a response "trigger".
    deflection_angle = 0
    #机器人应该转的角度.

    # The 80 is from half the X res, the 60 is from half the Y res. The
    # equation below is just computing the angle of a triangle where the
    # opposite side of the triangle is the deviation of the center position
    # from the center and the adjacent side is half the Y res. This limits
    # the angle output to around -45 to 45. (It's not quite -45 and 45).
    deflection_angle = -math.atan((center_pos-80)/60)
    #角度计算.80 60 分别为图像宽和高的一半，图像大小为QQVGA 160x120.
    #注意计算得到的是弧度值

    # Convert angle in radians to degrees.
    deflection_angle = math.degrees(deflection_angle)
    #将计算结果的弧度值转化为角度值
#------------------------------------------------------------------------------------------------
    # Now you have an angle telling you how much to turn the robot by which
    # incorporates the part of the line nearest to the robot and parts of
    # the line farther away from the robot for a better prediction.
    print(clock.fps()) # Note: Your OpenMV Cam runs about half as fast while
    # connected to your computer. The FPS should increase once disconnected.
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
     speed=9
     deg=0
     PackageSend()
    print(speed,deg)

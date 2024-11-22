# 5.blob_filtering - By: Janghun Hyeon - Fri Nov 22 2024

import sensor, image, time
from pyb import UART
import sensor
import time
from pyb import UART

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

clock = time.clock()
uart=UART(3,115200,timeout_char=200)
#while True:
#    uart.write("hello world")
#    time.sleep_ms(1000)
#    print('send')

# 1단계: Camera 스트리밍
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
clock = time.clock()


# 2단계: Blob 만들기
class Blob:
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    def rect(self):
        return (self.x, self.y, self.w, self.h)

    def area(self):
        return self.w * self.h

threshold = (200, 250, 130, 180, 0, 50) # 빨간색 임계값 설정

def find_blobs(img, threshold):
    width = img.width()
    height = img.height()
    blobs = []

    for y in range(0,height,5): # 간격 (5 pixel)을 두고 스캔
        for x in range(0,width,5):
            pixel = img.get_pixel(x, y)
            r, g, b = pixel

            if threshold[0]<r<threshold[1] and threshold[2]<g<threshold[3] and threshold[4]<b<threshold[5]:
                blobs.append(Blob(x,y,5,5)) # Blob 추가
    return blobs


# 4단계: Blob 병합
def merge_blobs(blobs, distance_threshold=40):
    merged_blobs = []

    while blobs:
        current_blob = blobs.pop(0)
        merged = False

        for merged_blob in merged_blobs:
            dx = abs(current_blob.x - merged_blob.x)
            dy = abs(current_blob.y - merged_blob.y)

            if dx <= distance_threshold and dy <= distance_threshold:
                x1 = min(current_blob.x, merged_blob.x)
                y1 = min(current_blob.y, merged_blob.y)
                x2 = max(current_blob.x + current_blob.w, merged_blob.x + merged_blob.w)
                y2 = max(current_blob.y + current_blob.h, merged_blob.y + merged_blob.h)

                merged_blob.x = x1
                merged_blob.y = y1
                merged_blob.w = x2-x1
                merged_blob.h = y2-y1

                merged = True
                break

        if not merged:
            merged_blobs.append(current_blob)

    return merged_blobs


# 5단계: Blob 크기 필터링
MIN_AREA_THRESHOLD = 200 # 필터링 임계값 설정
uart=UART(3,115200,timeout_char=200)
while(True):
    clock.tick()
    img = sensor.snapshot()
    blobs = find_blobs(img, threshold)

    merged_blobs = merge_blobs(blobs)

    for blob in merged_blobs:
        if blob.area()>max:
            

        if blob.area()>MIN_AREA_THRESHOLD:
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.x + blob.w // 2, blob.y + blob.h //2)
            # blob 정보 출력
            print("Blob [면적: %d, 중심 좌표: (%d, %d), 가로: %d, 세로: %d]" %(blob.area(), blob.x + blob.w//2, blob.y + blob.h//2, blob.w, blob.h))
            uart.write("(%d,%d)" % (blob.x + blob.w//2, blob.y + blob.h//2))
            time.sleep_ms(100)
    #print(clock.fps())

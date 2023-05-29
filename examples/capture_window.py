"""
Prerequirement
    pip install pywin32
    pip install opencv-python
    pip install numpy
    pip install psutil
"""

import numpy as np
import win32con
import win32gui
import win32ui
import win32process
import psutil
import ctypes
import cv2 as cv
import copy
import time
import signal
import sys
from threading import Thread, Lock

class WindowCapture:
    def __init__(self, exe_name):
        self.__stopping = False
        # find the handle for the window we want to capture
        self.hwnd = getWindowHandle(exe_name)
        self.window_name = getWindowTitleByHandle(self.hwnd)

        if not self.hwnd:
            raise Exception('Window not found: {}'.format(self.window_name))
        #
        self.__lock = Lock()
        t1 = Thread(target=self.__doWork)
        t1.start()
        self.__newestImage = np.array(np.zeros((100,100,3), dtype=np.uint8))
        self.__intermediaryImage = np.array(np.zeros((100,100,3), dtype=np.uint8))

        # get the window size
        window_rect = win32gui.GetWindowRect(self.hwnd)
        self.w = window_rect[2] - window_rect[0]
        self.h = window_rect[3] - window_rect[1]
        print(f"self.w: {self.w}; self.h: {self.h}")

        # account for the window border and titlebar and cut them off
        border_pixels = 8
        titlebar_pixels = 30
        #self.w = self.w - (border_pixels * 2)
        #self.h = self.h - titlebar_pixels - border_pixels
        self.cropped_x = border_pixels
        self.cropped_y = titlebar_pixels

        # set the cropped coordinates offset so we can translate screenshot
        # images into actual screen positions
        self.offset_x = window_rect[0] + self.cropped_x
        self.offset_y = window_rect[1] + self.cropped_y

    def get_screenshot(self):
        hwnd = self.hwnd

        wndc = win32gui.GetWindowDC(hwnd)
        imdc = win32ui.CreateDCFromHandle(wndc)
        # create a memory based device context
        memdc = imdc.CreateCompatibleDC()
        # create a bitmap object
        screenshot = win32ui.CreateBitmap()
        screenshot.CreateCompatibleBitmap(imdc, self.w, self.h)
        oldbmp = memdc.SelectObject(screenshot)
        # copy the screen into our memory device context
        memdc.BitBlt((0, 0), (self.w, self.h), imdc, (0, 0), win32con.SRCCOPY)
        memdc.SelectObject(oldbmp)
        bmpstr = screenshot.GetBitmapBits(True)
        img = np.frombuffer(bmpstr, dtype='uint8')
        win32gui.DeleteObject(screenshot.GetHandle())
        imdc.DeleteDC()
        win32gui.ReleaseDC(hwnd, wndc)
        memdc.DeleteDC()
        img.shape = (self.h, self.w, 4)
        return cv.cvtColor(img, cv.COLOR_BGRA2BGR)


    def __doWork(self):
        loop_time = 0
        while True:
            if self.__stopping:
                break
            try:
                self.__intermediaryImage = self.get_screenshot()
                self.__lock.acquire()
                self.__newestImage = self.__intermediaryImage
                self.__lock.release()
            except Exception as ex:
                continue
            try:
                fps = 1 / (time.time() - loop_time)
            except:
                pass
            #print(f'Raw FPS {fps}', flush=True)
            loop_time = time.time()


    def GetLatestImage(self):
        self.__lock.acquire()
        copyImage = copy.copy(self.__newestImage)
        self.__lock.release()
        return copyImage

    def stop(self):
        self.__stopping = True

def signal_handler(signal, frame):
    print("Stopping")
    windowCap.stop()
    sys.exit(0)

EnumWindows = ctypes.windll.user32.EnumWindows
EnumWindowsProc = ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int))
GetWindowText = ctypes.windll.user32.GetWindowTextW
GetWindowTextLength = ctypes.windll.user32.GetWindowTextLengthW
IsWindowVisible = ctypes.windll.user32.IsWindowVisible

def getProcessIDByName(executable):
    app_pids = []
    process_name = executable

    for proc in psutil.process_iter():
        if process_name in proc.name():
            app_pids.append(proc.pid)

    return app_pids

def get_hwnds_for_pid(pid):
    def callback(hwnd, hwnds):
        #if win32gui.IsWindowVisible(hwnd) and win32gui.IsWindowEnabled(hwnd):
        _, found_pid = win32process.GetWindowThreadProcessId(hwnd)

        if found_pid == pid:
            hwnds.append(hwnd)
        return True
    hwnds = []
    win32gui.EnumWindows(callback, hwnds)
    return hwnds

def getWindowTitleByHandle(hwnd):
    length = GetWindowTextLength(hwnd)
    buff = ctypes.create_unicode_buffer(length + 1)
    GetWindowText(hwnd, buff, length + 1)
    return buff.value

def getWindowHandle(executable):
    pids = getProcessIDByName(executable)

    print(pids)

    for i in pids:
        hwnds = get_hwnds_for_pid(i)
        for hwnd in hwnds:
            if IsWindowVisible(hwnd):
                print(hwnd)
                return hwnd

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)

    windowCap = WindowCapture(sys.argv[1])
    loop_time = 0
    while True:
        try:
            try:
                fps = 1 / (time.time() - loop_time)
            except:
                pass
            loop_time = time.time()
            #print(f'FPS {fps}', flush=True)
            img = windowCap.GetLatestImage()
            if img is None:
                continue
            #time.sleep(5//100)

            # Overlay FPS
            cv.putText(img, f"FPS: {fps:.2f}", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv.imshow("hi", img)
            if cv.waitKey(1) & 0xFF == ord('q'):
                raise KeyboardInterrupt
        except KeyboardInterrupt:
                print("Stopping...")
                windowCap.stop()
                break
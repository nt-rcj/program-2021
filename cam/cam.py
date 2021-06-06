#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
[summary]
  cam remapping test
[description]
  -
"""

import argparse
import copy

import cv2 as cv

import math
import numpy as np
import random


def get_args():
    """
    [summary]
        引数解析
    Parameters
    ----------
    None
    """

    parser = argparse.ArgumentParser()

    parser.add_argument("--width", help='capture width', type=int, default=960)
    parser.add_argument(
        "--height", help='capture height', type=int, default=540)

    args = parser.parse_args()

    return args


def main():
    """
    [summary]
        main()
    Parameters
    ----------
    None
    """
    # 引数解析 #################################################################
    args = get_args()
    cap_width = args.width
    cap_height = args.height

    # カメラ準備 ###############################################################
    cap = cv.VideoCapture('image/1.mp4')
    cap.set(cv.CAP_PROP_FRAME_WIDTH, cap_width)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, cap_height)

    while True:
        # カメラキャプチャ #####################################################
        ret, frame = cap.read()
        if not ret:
            print('cap.read() error')
            break

        # 極座標変換
        #rotate_frame = cv.rotate(frame, cv.ROTATE_90_CLOCKWISE)
        #lin_polar_image = cv.warpPolar(rotate_frame, (150, 500), (120, 160), 240, cv.INTER_CUBIC + cv.WARP_FILL_OUTLIERS + cv.WARP_POLAR_LINEAR)
        #print(lin_polar_image)

        # 画像データ読込
        h, w = frame.shape[:2]

        #マップ生成
        map_x = np.zeros((h, w), dtype=np.float32)
        map_y = np.zeros((h, w), dtype=np.float32)
        pitch = 50

        # 上下左右反転
        for x in range(h):
            map_x[x,:] = [(x*463700)/(140200+math.sqrt(x**2+28900)) for x in range(w)]
        for x in range(w):
            map_y[:,y] = [(y*463700)/(140200+math.sqrt(y**2+28900)) for y in range(h)]         



        # リマップ
        dst = cv.remap(frame, map_x, map_y, cv.INTER_CUBIC)

        # トリミング/向き調整
        #lin_polar_crop_image = copy.deepcopy(lin_polar_image[0:500, 15:135])
        #lin_polar_crop_image = lin_polar_crop_image.transpose(1, 0, 2)[::-1]

        # 逆変換(リニア)
        #flags = cv.INTER_CUBIC + cv.WARP_FILL_OUTLIERS + cv.WARP_POLAR_LINEAR + cv.WARP_INVERSE_MAP
        #linear_polar_inverse_image = cv.warpPolar(dst, (960, 540), (120, 160), 240, flags)

        # 描画
        cv.imshow('ORIGINAL', frame)
        cv.imshow('POLAR', dst)
        #cv.imshow('linear_polar_inverse_image', linear_polar_inverse_image)

        # キー入力(ESC:プログラム終了) #########################################
        key = cv.waitKey(50)
        if key == 27:  # ESC
            break

    cap.release()
    cv.destroyAllWindows()


if __name__ == '__main__':
    main()

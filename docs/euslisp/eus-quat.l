#!/usr/bin/env roseus

;; eusでのクオータニオンの扱いのまとめ
;; 参考: https://github.com/jsk-ros-pkg/euslisp-docs/blob/master/docs/jskeus/jmanual.md

;; ROSのgeometry msgのqruaternionはxyzwでeusのquaternionはwxyz



;; 1. 行列操作

;; 行列の定義 --> #2f((...)(...)..(...))
(setq m1 #2f((1 2 0) (3 2 1) (0 2 1)))

;; 逆行列の計算
(inverse-matrix m1) ;; #2f((0.0 0.333333 -0.333333) (0.5 -0.166667 0.166667) (-1.0 0.333333 0.666667))

;; 疑似逆行列の計算
(pseudo-inverse-org m1) ;; #2f((1.387779e-16 0.333333 -0.333333) (0.5 -0.166667 0.166667) (-1.0 0.333333 0.666667))

;; 対角行列の生成 (#fベクトルが引数)
(setq m2 (diagonal #f(1 2 3)))

;; マイナー操作 (minor-matrix m i j)で行列mからi行とj列を除く．復習: eusではindexは0開始
(setq m3 (minor-matrix m2 0 1)) ;; indexは0開始

;; 行列のlog
;; (matrix-log m1) ;;#f(0.389818 0.0 0.389818) ;; なんで？？？

;; 2. クオータニオン操作

;; クオータニオンの定義
(setq q1 #f(1.0 0.0 0.0 0.0)) ;; ★重要★　#f(w x y z) の順で定義

;; クオータニオンを回転行列に変換
(setq r1 (quaternion2matrix q1)) ;; -> #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))

;; 他の例
(setq q2 #f(0.7071 0.0 0.7071 0.0))
(setq r2 (quaternion2matrix q2))


;; 回転行列をクオータニオンに変換
(setq q3 (matrix2quaternion r2))

;; #3. ROSでのquaternionでのTFの操作
(load "models/arrow-object.l")
(setq *arrow1* (arrow :pos #f(0 0 0) :rot (unit-matrix 3)))
(objects (list *arrow1*))
(send *irtviewer* :draw-object)
;; クオータニオンの作成

;; 回転行列の生成


;; RPY --> quaternion
;; (ros::create-quaternion-from-rpy R P Y)
(setq q4 (ros::create-quaternion-from-rpy 0 0 0))
(setq q5 (ros::create-quaternion-from-rpy 0 0 pi/2))

;; RPY -> matrix
;; (rpy-matrix R P Y)
(setq m4 (rpy-matrix 0 0 0))
(setq m5 (rpy-matrix 0 0 pi/2))
(setq m5 (rpy-matrix 0 0 pi/2))
;; matrix -> RPY (*remark* :2組ある)
;; (rpy-angle mat) ;; 行列から2組のRPYのペアを出力する RPYの順
(setq t4 (elt (rpy-angle m4) 0) ;;((0.0 0.0 0.0) (3.14159 -3.14159 3.14159))
(setq t5 (elt (rpy-angle m5) 0) ;;((0.0 0.0 0.0) (3.14159 -3.14159 3.14159))


(setq r5 (quaternion2matrix q5))
(send *arrow1* :newcoords (make-coords :rot r5))
(send *irtviewer* :draw-object)



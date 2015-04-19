
import math
import OpenGL
OpenGL.ERROR_CHECKING = True
from OpenGL.GL import*
from OpenGL.GLUT import*


#/*----------------------------------------------------------------------------
#名前：draw_cylinder
#説明：ワイヤモデルの円柱の描画
#引数>
#radius：半径[m]
#div：分割数
#width：円柱の幅（高さ）[m]
#戻り値>
#void
#----------------------------------------------------------------------------*/
def drawCylinder(radius, div, width):
	dth = 2*math.pi/div
	# 底面の描写
	glBegin(GL_LINE_LOOP)
	for th in range(0, 2*math.pi, dth):
		x = radius * cos(th)
		z = radius * sin(th)
		glVertex3f(x, 0, z) # 描画したい円周上の頂点を並べる
	glEnd()

	# 上面の描写
	glBegin(GL_LINE_LOOP)
	for th in range(0, 2*math.pi, dth):
		x = radius * cos(th)
		z = radius * sin(th)
		glVertex3f(x, width, z)  # 描画したい円周上の頂点を並べる
	glEnd()

	# 底面と上面の点を結ぶ
	glBegin(GL_LINES)
	for th in range(0, 2*math.pi, dth):
		x = radius * cos(th)
		z = radius * sin(th)
		glVertex3f(x, 0, z)              # 底面と上面の点を結ぶ
		glVertex3f(x, width, z)
	glEnd()


#/*------------------------------------------------------------------------------
#名前：drawCylinderSurface
#説明：サーフェスモデルの円柱の描画
#補足：GL_POLYGONは処理が重いらしいので,GL_TRIANGLE_STRIPを使用
#引数>
#radius：半径[m]
#div：分割数
#width：円柱の幅（高さ）[m]
#戻り値>
#void
#-------------------------------------------------------------------------------*/
def drawCylinderSurface(radius, div, width):
	dth = 2*math.pi/div
	# 底面の描写(表面あり,線はなし
	glBegin(GL_TRIANGLE_STRIP)
	for th in range(0, 2*math.pi, dth):
		x = radius * cos(th)
		z = radius * sin(th)
		x2 = radius * cos(th + dth)
		z2 = radius * sin(th + dth)
		glVertex3f(x, 0.0, z)          # 描画したい円周上の頂点を並べる
		glVertex3f(x2, 0.0, z2)
		glVertex3f(0.0, 0.0, 0.0)
	glEnd()

	# 上面の描写(表面あり．線はなし)
	glBegin(GL_TRIANGLE_STRIP)# 処理速度が遅い
	for th in range(0, 2*math.pi, dth):
		x = radius * cos(th)
		z = radius * sin(th)
		x2 = radius * cos(th + dth)
		z2 = radius * sin(th + dth)
		glVertex3f(x, width, z)          # 描画したい円周上の頂点を並べる
		glVertex3f(x2, width, z2)
		glVertex3f(0.0, width, 0.0)
	glEnd()


	# 底面と上面の点を結ぶ
	glBegin(GL_QUADS)
	for th in range(0, 2*math.pi, dth):
		x = radius * cos(th)
		z = radius * sin(th)
		x2 = radius * cos(th + dth)
		z2 = radius * sin(th + dth)
		glVertex3f(x, 0, z)              # 底面と上面の点を結ぶ
		glVertex3f(x, width, z)
		glVertex3f(x2, width, z2)
		glVertex3f(x2, 0, z2)
	glEnd()

#/*------------------------------------------------------------------------------
#名前：drawSphere
#説明：球の描画
#引数>
#sphere_size:球の半径[m]
#戻り値>
#void
#-------------------------------------------------------------------------------*/
def drawSphere(sphereSize):
	glutSolidSphere(sphereSize, 10, 5)


#/*------------------------------------------------------------------------------
#名前：drawCircle
#説明：3次元空間のx-y平面に原点(0,0,0)を中心に半径radiusの円を描く
#引数>
#radius：半径[m]
#div: 細かさ（分割数）
#戻り値>
#void
#-------------------------------------------------------------------------------*/
def drawCircle (radius, div):
	dth = 2*math.pi/div
	glBegin(GL_POLYGON)
	for th in range(0, 2*math.pi, dth):
		x = radius * cos(th)
		y = radius * sin(th)
		glVertex3f(x, y, 0.0)          # 描画したい円周上の頂点を並べる
	glEnd()

#/*------------------------------------------------------------------------------
#名前：drawCircleEdge
#説明：3次元空間のx-y平面に原点(0,0,0)を中心に半径radiusの円の円周を描く
#引数>
#radius：半径[m]
#div: 細かさ（分割数）
#戻り値>
#void
#-------------------------------------------------------------------------------*/
def drawCircleEdge (radius, div):
	dth = 2*math.pi/div
	glBegin(GL_LINE_LOOP)
	for (float th = 0.0  th <= 2*M_PI  th += dth) {
		x = radius * cos(th)
		y = radius * sin(th)
		glVertex3f(x, y, 0.0)          # 描画したい円周上の頂点を並べる
	glEnd()


#/*------------------------------------------------------------------------------
#名前：drawGrandAxis
#説明：座標軸の描画
#引数>
#void
#戻り値>
#def
#-------------------------------------------------------------------------------*/
def drawGrandAxis ():

	#  X軸の描画 --------------------------------
	#  描画を赤色に設定
	glColor4d(1.0, 0.0, 0.0, 1.0)
	#  軸の描画
	glBegin(GL_LINES)
		glVertex3d(0.0, 0.0, 0.0)
		glVertex3d(1000.0, 0.0, 0.0)
	glEnd()

	#  Y軸の描画 --------------------------------
	#  描画を緑色に設定
	glColor4d(0.0, 1.0, 0.0, 1.0)
	#  軸の描画
	glBegin(GL_LINES)
		glVertex3d(0.0, 0.0, 0.0)
		glVertex3d(0.0, 1000.0, 0.0)
	glEnd()

	#  Z軸の描画 --------------------------------
	#  描画を青色に設定
	glColor4d(0.0, 0.0, 1.0, 1.0)
	#  軸の描画
	glBegin(GL_LINES)
		glVertex3d(0.0, 0.0, 0.0)
		glVertex3d(0.0, 0.0, 1000.0)
	glEnd()


#/*------------------------------------------------------------------------------
#名前：drawGrand
#説明：地面のグリッド描画
#補足：width,depthは偶数であること
#引数>
#width:横方向（y軸方向）の長さ[m]
#depth:縦方向（x軸方向）の長さ[m]
#initerval:一マスの長さ[m]
#戻り値>
#void
#-------------------------------------------------------------------------------*/
def drawGrand (width, depth, interval):
	#  幅，高さの大きさを描画に使うため半分にする
	glBegin(GL_LINES)			#  地面のグリッド
	#  y軸と平行なグリッド線を引く
	for i in range(-width/2.0, width/2.0, interval):
		glVertex3d(i, -depth, 0.0)
		glVertex3d(i, depth, 0.0)

	#  x軸と平行なグリッド線を引く(y軸正方向)
	for i in range(-depth/2.0, depth/2.0, interval):
		glVertex3d(-width, i, 0.0)
		glVertex3d(width, i, 0.0)
	glEnd()

}

#/*------------------------------------------------------------------------------
#名前：draw_grand2
#説明：移動体の周りだけ地面のグリッド描画（移動体から遠ざかるほど薄くなる）
#引数>
#width:横方向（y軸方向）の長さ[m]
#depth:縦方向（x軸方向）の長さ[m]
#initerval:一マスの長さ[m]
#power:透明の成り具合
#戻り値>
#void
#-------------------------------------------------------------------------------*/
def drawGrand2 (width, depth, interval, power):
	# 透明処理の設定
	glEnable (GL_BLEND)
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

	glBegin(GL_LINES)#  地面のグリッド
	for i in range(-depth/2.0, depth/2.0, interval):
		# x方向
		for j in range(-width/2.0, width/2.0, interval):
			glColor4f(1.0, 1.0, 1.0, 1.0 - pow(abs(i/(width/2.0)),power) - pow(abs(j/(width/2.0)),power) )
			glVertex3d(j, i, 0.0)
			glVertex3d(j+interval, i, 0.0 )

	for i in range(-width/2.0, width/2.0, interval):
		# y方向
		for j in range(-depth/2.0, depth/2.0, interval):
			glColor4f(1.0, 1.0, 1.0, 1.0-pow(abs(i/(depth/2.0)),power)-pow(abs(j/(depth/2.0)),power) )
			glVertex3d(i, j, 0.0 )
			glVertex3d(i, j+interval , 0.0 )
	glEnd()
	glDisable (GL_BLEND)


#/*------------------------------------------------------------------------------
#名前：draw_grand_radial
#説明：自機からのxy平面上の放射状の線
#補足：デフォルトでは原点周りのxy平面上なので，3次元空間に描くときは，まず並進，回転行列でモデルビューを変更してから，この円を描く
#引数>
#radius:表示する半径[m]
#degree:放射状線間の角度[degree]
#location：自機の位置姿勢情報
#戻り値>
#void
#-------------------------------------------------------------------------------*/
def drawGrandRadial (radius, degree):
# 	glLineWidth(1.0)	# 線幅の決定
	for theta in range(0.0, 2.0*math.pi, math.radians(degree))
	glBegin(GL_LINES)
		glVertex3f(0.0 , 0.0 , 0.0)		# スタート座標
		glVertex3f(0.0+radius*cos(theta) , 0.0+radius*sin(theta) , 0.0)		# ゴール座標
	glEnd()

#/*------------------------------------------------------------------------------
#名前：drawGrandEqualDistance
#説明：自機からのxy平面上の等距離線
#補足：デフォルトでは原点周りのxy平面上なので，3次元空間に描くときは，まず並進，回転行列でモデルビューを変更してから，この円を描く
#引数>
#radius:表示する半径[m]
#initerval:各線間の長さ[m]
#戻り値>
#void
#-------------------------------------------------------------------------------*/
def drawGrandEqualDistance (radius, interval):
# 	glLineWidth(1.0)	# 線幅の決定
	delta = math.radians(5)
	for r in range(interval, radius, interval):
		glBegin(GL_LINE_LOOP)
		for theta in range(0.0, 2*math.pi, delta):
			glVertex3f(0.0+r*cos(theta) , 0.0+r*sin(theta) , 0.0)		# 繋げたい点を並べる
		glEnd()

#/*------------------------------------------------------------------------------
#名前：drawLine
#説明：直線の描画
#補足：
#引数>
#startPoint[3]:始点
#endPoint[3]:終点
#戻り値>
#void
#-------------------------------------------------------------------------------*/
def drawLine(startPoint, endPoint):
	glBegin(GL_LINES)
	glVertex3f(startPoint[0], startPoint[1], startPoint[2])
	glVertex3f(endPoint[0], endPoint[1], endPoint[2])
	glEnd()

#/************************************************************
#名前： drawEye
#説明： 視点位置の描画
#		distance=d(?)

#引数>
#width：画面の横[pixel]
#heigh:画面の縦[pixel]
#戻り値>
#void
#**************************************************************/
def drawEye(width, height, h_gakaku):

	#  アスペクト比の計算
	aspect = width / height
	distance = (width)/math.tan(math.radiuns(h_gakaku))
	y = 0.1
	z = y/aspect

	glPushMatrix()				#  座標系の保存
	#  線の太さと色を設定
	glLineWidth(3.0)
	glColor3f(1.0, 0.0, 0.0)
	# 原点の描画******************************************************
	glPointSize (5)
	glBegin (GL_POINTS)
	glVertex3d (0.0, 0.0, 0.0)
	glEnd ()
	# 四角錐部の描画***************************************************
	glBegin (GL_LINES)
	# 四角錐の側面の4辺
	glVertex3d (0.0, 0.0, 0.0)
	glVertex3d (distance, y, -z)

	glVertex3d (0.0, 0.0, 0.0)
	glVertex3d (distance, y, z)

	glVertex3d (0.0, 0.0, 0.0)
	glVertex3d (distance, -y, z)

	glVertex3d (0.0, 0.0, 0.0)
	glVertex3d (distance, -y, -z)

	# 四角錐の底面の4辺
	glVertex3d (distance, y, -z)
	glVertex3d (distance, y, z)

	glVertex3d (distance, y, z)
	glVertex3d (distance, -y, z)

	glVertex3d (distance, -y, z)
	glVertex3d (distance, -y, -z)

	glVertex3d (distance, -y, -z)
	glVertex3d (distance, y, -z)
	glEnd ()

	glPopMatrix()				#  座標系の復元

}


#/*-----------------------------------------------------------
#名前:drawLayerOnScreen
#説明:画面いっぱいに薄い赤のレイヤーを表示
#引数＞
#width:	画面横幅
#height:	画面縦幅
#alpha:	透明度
#戻り値＞
#void
#------------------------------------------------------------*/
def drawLayerOnScreen(width, height, alpha):

	glPushMatrix() # 各種行列を退避
	glLoadIdentity()
	glMatrixMode(GL_PROJECTION) # 2Dの並行投影を設定

	glPushMatrix()
	glLoadIdentity()
	gluOrtho2D(0, width, 0, height)
	glEnable(GL_BLEND)			# ブレンドの有効化
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)# ブレンドの仕方
	glColor4d(1.0, 0.0, 0.0, alpha)	# 描画色の設定
	glBegin (GL_QUADS)
	glVertex3d (0.0, 0.0, 0.0)
	glVertex3d (width, 0.0, 0.0)
	glVertex3d (width, height, 0)
	glVertex3d (0.0, height, 0)
	glEnd ()
	glDisable(GL_BLEND)
	glPopMatrix()

	glMatrixMode(GL_MODELVIEW)
	glPopMatrix()# もとの状態にもどる



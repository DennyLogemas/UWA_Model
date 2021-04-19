{*VICON BodyLanguage (tm)*}

{*====================================================================================================================================*}
{*====================================================UWA UPPER BODY DYNAMIC MODEL====================================================*}
{*====================================================================================================================================*}

{*
	1. Program-Name Creation-Date
	UWAUBDynamic.mod    30/June/2016
	2. Original-Author (Email-Address)
	Multiple Contributors
	3. Last-Updated-By (Email-Address)
	Dan Cottam (daniel.cottam@research.uwa.edu.au)
	4. Short-Description
	The generic UWA upper body dynamic bodybuilder model
	5. Notes
	Please see documents on Sharepoint named "Changes to UWA Lower and Upper Body Models (June, 2016)" and Version 1.1 UWA Model Changes (August 2016) for information on model changes.
	6. Modification-History
	Version  Author     Date                 Change
	1.1       DC    10/August/2016   See Version 1.1 UWA Model Changes document on Sharepoint
*}

{*Start of macro section*}
{*==============================================================*}

{* ============ MACRO SUBSTITUTE4 ============*}
macro SUBSTITUTE4(p1,p2,p3,p4)
{* Replaces any point missing from set of four fixed in a segment*}
s234 = [p3,p2-p3,p3-p4]
p1V = Average(p1/s234)*s234
s341 = [p4,p3-p4,p4-p1]
p2V = Average(p2/s341)*s341
s412 = [p1,p4-p1,p1-p2]
p3V = Average(p3/s412)*s412
s123 = [p2,p1-p2,p2-p3]
p4V = Average(p4/s123)*s123
p1 = (p1+p1V)/2 ? p1 ? p1V
p2 = (p2+p2V)/2 ? p2 ? p2V
p3 = (p3+p3V)/2 ? p3 ? p3V
p4 = (p4+p4V)/2 ? p4 ? p4V
OUTPUT(p1,p2,p3,p4)
endmacro

{* ============ MACRO DRAWGLOBAL ============*}
Macro DrawGlobal(ScaleFactor)
{* draws in the global coordinate system *}
Oglobal={0,0,0}
Xglobal=ScaleFactor*{1,0,0}
Yglobal=ScaleFactor*{0,1,0}
Zglobal=ScaleFactor*{0,0,1}
GlobalSystem=[Oglobal,Yglobal-Oglobal,Xglobal-Oglobal,yzx]
OUTPUT(Xglobal,Yglobal,Zglobal,Oglobal)
EndMacro

{* ============ MACRO DRAWSEGMENT ============*}
Macro DrawSegment(segm)
{* draws each individual segment coordinate system *}
O#segm={0,0,0}*segm
X#segm=O#segm+80*1(segm) 
Y#segm=O#segm+80*2(segm) 
Z#segm=O#segm+80*3(segm) 
OUTPUT(O#segm,X#segm,Y#segm,Z#segm)
EndMacro

{* ============ MACRO LINVELACC ============*}
macro LINVELACC(Point)
{*When called, this macro calculates the linear velocity in m/s and the linear acceleration in m/s^2 of a
point, using numerical differentiation.  For numerical differentiation, reference one of the following:
Hildebrand, F.B. (1974).  Introduction to Numerical Analysis, 2nd Edition, pp.111
Kreyszig, Erwin (1983).  Advanced Engineering Mathematics, 5th Edition, pp.793
Yakowitz, Sydney and Szidarovsky, Ferenc (1989).  An Introduction to Numerical Computations, 2nd Edition, pp.185*}

{* Define time difference between samples *}
FrameTimeLength=1/$SamplingRate

{** ---- 2nd order finite difference version ---- **}

Point#LVel=((Point[1]-Point[-1])/(2*FrameTimeLength))/1000
Point#LAcc=((Point#LVel[1]-(2*Point#LVel[0])+(Point#LVel[-1]))/(FrameTimeLength*FrameTimeLength))
Point#AbsoluteVel = SQRT((Point#LVel(1)*Point#LVel(1))+(Point#LVel(2)*Point#LVel(2))+(Point#LVel(3)*Point#LVel(3)))
Point#AbsoluteAcc = SQRT((Point#LAcc(1)*Point#LAcc(1))+(Point#LAcc(2)*Point#LAcc(2))+(Point#LAcc(3)*Point#LAcc(3)))

{*
{** ---- 4th order finite difference version ---- **}
Point#LVel=((Point[-2]-(8*Point[-1])+(8*Point[1])-Point[2])/(12*FrameTimeLength))/1000
Point#LAcc=((Point#LVel[-2]-(8*Point#LVel[-1])+(8*Point#LVel[1])-Point#LVel[2])/(12*FrameTimeLength))
Point#AbsoluteVel = sqrt((Point#LVel(1)*Point#LVel(1))+(Point#LVel(2)*Point#LVel(2))+(Point#LVel(3)*Point#LVel(3)))
Point#AbsoluteAcc = sqrt((Point#LAcc(1)*Point#LAcc(1))+(Point#LAcc(2)*Point#LAcc(2))+(Point#LAcc(3)*Point#LAcc(3)))
*}

output(Point#LVel,Point#LAcc,Point#AbsoluteVel,Point#AbsoluteAcc)
param(FrameTimeLength)
endmacro

{* ============ MACRO ANGVELACC ============*}
Macro ANGVELACC(child,parent,Joint)
{* When called, this macro calculates the angular velocity in rad/s and the angular acceleration in rad/s^2
at a joint, using numerical differentiation.  Output is also converted and displayed as degrees as an option. *}
{* Angular velocity is calculated according to JJ Craig (need to insert ref)*}

FrameTimeLength=1/$SamplingRate
pi=3.1415927

{* Calculate unit vectors of child axes in parent coordinate system *}
{* Pete Mills' Child to Parent rotation matrix *}
{*
ChildTranslatedToParent = O#parent + ATTITUDE(child)
ChildInParent = ChildTranslatedToParent/parent
OChildInParent= {0,0,0}*ChildInParent
XChildInParent= OChildInParent + 1(ChildInParent)
YChildInParent= OChildInParent + 2(ChildInParent)
ZChildInParent= OChildInParent + 3(ChildInParent)
*}

{* David Lloyd's Child to Parent rotation matrix *}
XChildInParent= (({1,0,0}*child)/parent)-(({0,0,0}*child)/parent)
YChildInParent= (({0,1,0}*child)/parent)-(({0,0,0}*child)/parent)
ZChildInParent= (({0,0,1}*child)/parent)-(({0,0,0}*child)/parent)

{* Differentiate unit vectors of child axes in parent coordinate system *}
drXChildInParent = ((XChildInParent[1] - XChildInParent[-1])/(2*FrameTimeLength))
drYChildInParent = ((YChildInParent[1] - YChildInParent[-1])/(2*FrameTimeLength))
drZChildInParent = ((ZChildInParent[1] - ZChildInParent[-1])/(2*FrameTimeLength))
output(drXChildInParent,drYChildInParent,drZChildInParent)

{* Multiply derivative matrix terms by rotation matrix terms to give angular velocity vector *}
AVelXChildInParent = (drXChildinParent(3)*XChildinParent(2) + drYChildinParent(3)*YChildinParent(2) + drZChildinParent(3)*ZChildinParent(2))
AVelYChildInParent = -(drXChildinParent(3)*XChildinParent(1) + drYChildinParent(3)*YChildinParent(1) + drZChildinParent(3)*ZChildinParent(1))
AVelZChildInParent = (drXChildinParent(2)*XChildinParent(1) + drYChildinParent(2)*YChildinParent(1) + drZChildinParent(2)*ZChildinParent(1))
Joint#AngVel = {AVelXChildInParent,AVelYChildInParent,AVelZChildInParent}

{* Differentiate angular velocity vector to give angular acceleration *}
Joint#AngAcc=((Joint#AngVel[1]-Joint#AngVel[-1])/(2*FrameTimeLength))

{* Convert rads based derivatives to degs and output to c3d *}
Joint#AngVelDeg=Joint#AngVel*(180/pi)
Joint#AngAccDeg=Joint#AngAcc*(180/pi)
param(FrameTimeLength)
endmacro

{* ============ MACRO JOINTPOWER ============*}
Macro JOINTPOWER(child,parent,Joint)
{* This macro calculates individual joint power terms. Joint angular velocity
must be calculated prior to running this macro. Power terms should be calculated prior to 
changing the signs of the joint moments to clinical/functionally relevant conventions *}

{* Transform moment into parent coordinate system *}
ChildAttitude = ATTITUDE(child)
ParentAttitude = ATTITUDE(parent)
MomentInGlobal = Joint#Moments*ChildAttitude
MomentInParent = MomentInGlobal/ParentAttitude

{* Multiply moments by angular velocity to derive joint power terms *}
PowerTermsX = MomentInParent(3)*Joint#AngVel(3) 		{* Flex/Ext *}
PowerTermsY = MomentInParent(1)*Joint#AngVel(1)  		{* Add/Abd  *}
PowerTermsZ = MomentInParent(2)*Joint#AngVel(2) 		{* Int/Ext rot    *}

PowerTermsXNorm =  PowerTermsX/$Bodymass
PowerTermsYNorm =  PowerTermsY/$Bodymass
PowerTermsZNorm =  PowerTermsZ/$Bodymass

Joint#PowerTerms = {PowerTermsX,PowerTermsY,PowerTermsZ}
Joint#PowerTermsNorm = {PowerTermsXNorm,PowerTermsYNorm,PowerTermsZNorm}

{* Output joint power to c3d *}
OUTPUT(Joint#PowerTerms)
OUTPUT(Joint#PowerTermsNorm)
endmacro

{* ============ MACRO YXY decomposition ============ *}
{* This macro took bloody ages to get right because Bodybuilder's sine and    *}
{* cosine functions have a mind of their own. It is based on spherical coord  *} 
{* system and uses a YXY decomposition to calculate the                       *}	
{* yfirst = pole angle (rotation about parent Y axis) or plane of elevation   *}
{*      x = elevation, rotation about posterior-anterior x-axis of child      *}
{*   ysec = rotation (twist) about long axis                                  *}
{* These series of angles are the ISB shoudlder standard. This doesn't work   *}
{* for all shoulder movements and you may have to use other decompositions.   *}

Macro RotYXYType1(child,parent,joint)
{* David Lloyd's Child to Parent rotation matrix *}
XChildInParent= (({1,0,0}*child)/parent)-(({0,0,0}*child)/parent)
YChildInParent= (({0,1,0}*child)/parent)-(({0,0,0}*child)/parent)
ZChildInParent= (({0,0,1}*child)/parent)-(({0,0,0}*child)/parent)
r11 = XChildInParent(1)
r21 = XChildInParent(2)
r31 = XChildInParent(3) 
r12 = YChildInParent(1)
r22 = YChildInParent(2)
r32 = YChildInParent(3)

x = acos(r22)
IF x < 0
	x = -x
ENDIF
sx = sin(x)

{* yfirst is the first rotation about y axis, or alpha angle in YXY decomp *}
sy = r12/sx
cy = r32/sx 
y = atan2(sy, cy)

IF x==0 OR x==180
	y = 0
	{* set x-axis of intermediate pre-twist CS to parent x-axis *}
	r32=1
	r12=0
ENDIF
yfirst=y

{* setup imtermediate segments to calc twist rotation *}
{* Dummy0 is the final child segment position relative to parent
   Dummy1 is the intermediate pre-twist CS of child rel to parent
   Dummy2 is 90 deg twisted intermediate pre-twist CS of child rel to parent *}
Dummy0 =[{0,0,0},{r12, r22, r32},{r11, r21, r31},yzx]
Dummy1 =[{0,0,0},{r12, r22, r32},{-r32,0,r12},yzx]
twist1 = atan2(COMP(Dummy1(1),Dummy0(3)),COMP(Dummy1(3),Dummy0(3)))
{*
Dummy2 =[{0,0,0},{r12, r22, r32},{ r32,0,-r12},yxz]
twist2 = atan2(COMP(Dummy2(1),Dummy0(3)),COMP(Dummy2(3),Dummy0(3)))
*}

{* Same as above except referred to the global CS for viewing *}
{*
Dummy0 =[{0,0,0},{r12, r22, r32}*parent,{r11, r21, r31}*parent,yzx]
Dummy1 =[{0,0,0},{r12, r22, r32}*parent,{-r32,0,r12}*parent,yzx]
twist1 = atan2(COMP(Dummy1(1),Dummy0(3)),COMP(Dummy1(3),Dummy0(3)))
Dummy2 =[{0,0,0},{r12, r22, r32}*parent,{ r32,0,-r12}*parent,yxz]
twist2 = atan2(COMP(Dummy2(1),Dummy0(3)),COMP(Dummy2(3),Dummy0(3)))
*}

IF x==0 OR x==180
	ysec = acos(r11)
ELSIF
	ysec=twist1
ENDIF

joint#YXY = <yfirst, x, ysec>

output(joint#YXY)

endmacro

{* ============ MACRO TO MOVE POWER TO JCS ============================*}
{* Assumes that the JCS is a ZXY decomposition, i.e. Grood & Suntay    *}
Macro POWTOJCS(PowerIn, child, parent, Joint)

{* Move from psuedo JCS to XYZ *}
PowerXYZ={2(PowerIn),3(PowerIn),1(PowerIn)}

{* Creating Float, Child, and Parent Attitude *}
FloatAttitude = [{0,0,0}, 2(Child), -3(Parent), yxz]
ChildAttitude = ATTITUDE(Child)
ParentAttitude = ATTITUDE(Parent)

{* Move power, which are assumed to be parent CS respectively, to the Child CS respectively *}
PowerChild = (PowerXYZ*ParentAttitude)/ChildAttitude

{* Move moments and power, which are assumed to be in parent CS respectively, to the Float CS *}
PowerFloat = (PowerXYZ*ParentAttitude)/FloatAttitude

{* Move moments and power into JCS *}
PowerJCS#Joint = {3(PowerXYZ), 1(PowerFloat), 2(PowerChild)}

Output(PowerJCS#Joint)

endmacro

{* ============ MACRO TO MOVE MOMENTS TO JCS ==========================*}
{* Assumes that the JCS is a ZXY decomposition, i.e. Grood & Suntay    *}
Macro MOMTOJCS(Moments, child, parent, Joint)

{* Creating Float, Child, and Parent Attitude *}
FloatAttitude = [{0,0,0}, Child(2), -Parent(3), yxz]
ChildAttitude = ATTITUDE(Child)
ParentAttitude = ATTITUDE(Parent)

{* Move moments and power, which are assumed to be in child CS and parent CS
   respectively, to the Parent and Child CS respectively *}
MomentsParent = (Moments*ChildAttitude)/ParentAttitude

{* Move moments and power, which are assumed to be in child CS and parent CS
   respectively, to the Float CS *}
MomentsFloat = (Moments*ChildAttitude)/FloatAttitude

{* Move moments and power into JCS *}
MomentJCS#Joint = {MomentsParent(3), MomentsFloat(1), Moments(2)}

Output(MomentJCS#Joint)

endmacro


{*======================*}
Macro CreateRelPoint(PGlob,segm,NewPointName)
{*Creates a new point relative to a local segement*}
PLocal=PGlob/segm
PLocalYValue=PLocal(2)
PLocalYPoint=PLocalYValue*{0,1,0}
NewPointName=PLocalYPoint*segm
EndMacro


{*End of macro section*}
{*==============================================================*}

{*==============================================================*}
{* START Initialisations*}
{*==============================================================*}

{*   Optional Points   *}
{*=====================*}

{* Head & Thorax Markers*}

	OptionalPoints(LFHD, RFHD, LBHD, RBHD)
	OptionalPoints(C7, T10, CLAV, STRN)
	OptionalPoints(VTR1,VTR2,VTR3,VTR4)
	
{* Left Arm Markers*}

	OptionalPoints(LACR, LASH, LPSH) 
	OptionalPoints(LACR1, LACR2, LACR3) 
	OptionalPoints(LUA1, LUA2, LUA3, LUA4)
	OptionalPoints(dLUA1, dLUA2, dLUA3, dLUA4)
	OptionalPoints(LFA1, LFA2, LFA3, LFA4)
	OptionalPoints(LWRR, LWRU)
	OptionalPoints(LHNR, LHNU, LCAR)
	
{* Right Arm Markers*}

	OptionalPoints(RACR, RASH, RPSH)
	OptionalPoints(RACR1, RACR2, RACR3) 
	OptionalPoints(RUA1, RUA2, RUA3, RUA4)
	OptionalPoints(dRUA1, dRUA2, dRUA3, dRUA4)
	OptionalPoints(RFA1, RFA2, RFA3, RFA4)
	OptionalPoints(RWRR, RWRU)
	OptionalPoints(RHNR, RHNU, RCAR)
	
	
{* Pelvis Markers*}

	OptionalPoints(LPSI, RPSI, LASI, RASI)

{* Virtual Markers*}

	OptionalPoints($LEnHA1, $LEnHA2, $REnHA1, $REnHA2) 
	OptionalPoints($LEspHA1, $LEspHA2, $REspHA1, $REspHA2) 
	OptionalPoints($LSJCA,$LSJCB,$RSJCA,$RSJCB)
	OptionalPoints($LMELpointerRelTech,$LLELpointerRelTech,$RMELpointerRelTech,$RLELpointerRelTech)
	OptionalPoints($LWRRAnatRelTech,$LWRUAnatRelTech,$RWRRAnatRelTech,$RWRUAnatRelTech)
	OptionalPoints($LOptimalHip,$ROptimalHip)
	OptionalPoints($VTR1RelTech,$VTR2RelTech,$VTR3RelTech)
	OptionalPoints($VT10pointerRelTech,$VSTRNpointerRelTech,$VC7pointerRelTech)
	OptionalPoints($RUA1RelTech,$RUA2RelTech,$RUA3RelTech)
	OptionalPoints($LUA1RelTech,$LUA2RelTech,$LUA3RelTech)
	OptionalPoints($dRUA1RelTech,$dRUA2RelTech,$dRUA3RelTech)
	OptionalPoints($dLUA1RelTech,$dLUA2RelTech,$dLUA3RelTech)
	OptionalPoints($RFA1RelTech,$RFA2RelTech,$RFA3RelTech)
	OptionalPoints($LFA1RelTech,$LFA2RelTech,$LFA3RelTech)
	OptionalPoints($RiMEL,$RiLEL,$LeMEL,$LeLEL)
	OptionalPoints(feLEJC,feREJC)
	
{*Ball Markers*}

	OptionalPoints(Ball1, Ball2, Ball3, Ball4)

DrawGlobal(200)

{*=============================================================================*}

{* Anthropometry Values *}
{* Add the hierarchy, mass, and inertia to each segment. Default measures are for males. Obtained from
   Paolo de Leva, 1996, Journal of Biomechanics, Vol 29, pp 1223-1230. These have been adjusted since 
   HMgaitV2.mod due to x,y,z order error and a squaring error*}

{*Using Male Kinetics paramaters - this will return $KineticParameters = 1 in MP file*}
If $Female <> 1 and $Adult <> 0
	HeadLength=.1395	
	HeadMass=.0694
	HeadPos={0,0.4998,0}
	
	TrunkMass=0.4346
	TrunkPos={0,0.4862,0}
	TrunkRG={0.328*0.328,0.169*0.169,0.306*0.306}
	
	ShldWingMass=0.0798
	
	UpperArmMass=0.0271
	UpperArmPos={0,0.4228,0}
	UpperArmRG={0.285*0.285,0.158*0.158,0.269*0.269}
	
	ForearmMass=0.0162
	ForearmPos={0,0.5426,0}
	ForearmRG={0.276*0.276,0.121*0.121,0.265*0.265}
	
	HandMass=0.0061
	HandPos={0,0.2100,0}
	HandRG={0.628*0.628,0.401*0.401,0.513*0.513}
	$KineticParametersUB = 1

EndIf

{*Using Female Kinetics paramaters - this will return $KineticParameters = 2 in MP file*}
If $Female <> 0 and $Adult <> 0
	HandPos={0,0.7474,0}
	HeadMass=0.0668
	HeadPos={0,0.5159,0}
	
	TrunkMass=0.4257
	TrunkPos={0,0.5036,0}
	TrunkRG={0.307*0.307,0.147*0.147,0.292*0.292}	

	ShldWingMass=0.07725
	
	UpperArmMass=0.0255
	UpperArmPos={0,0.4246,0}
	UpperArmRG={0.278*0.278,0.148*0.148,0.260*0.260}
	
	ForearmMass=0.0138
	ForearmPos={0,0.5441,0}
	ForearmRG={0.261*0.261,0.094*0.094,0.257*0.257}
	
	HandMass=0.0056
	HandPos={0,0.2526,0}
	HandRG={0.531*0.531,0.335*0.335,0.454*0.454}
	$KineticParametersUB = 2
EndIf
PARAM($KineticParametersUB)


{* =================================================================================*}
{* 										Pelvis		 			 	       		    *}
{* =================================================================================*}
{* Uses HJC location of Orthotrak (Shea et al.1997 Gait and Posture 5,157) 	    *}

	If EXISTATALL(LPSI,RPSI)
		SACR=(LPSI+RPSI)/2
	EndIf
	OUTPUT(SACR)

	PELOrigin=(LASI+RASI)/2
	MidPelvis=(SACR+PELOrigin)/2
	OUTPUT(PELOrigin,MidPelvis)
	Pelvis = [PELOrigin,RASI-LASI,SACR-PELOrigin,zyx]

	If $OptimalHip==0
		InterASISDist=$InterASISdist
		If $InterASISdist == 0
			InterASISDist=DIST(LASI,RASI)
		EndIf
		aa = InterASISDist/2
		mm = $MarkerDiameter/2
		LHJC = {-(0.21*InterASISDist)-mm,
				 -(0.32*InterASISDist),
				 (-0.34*InterASISDist)}*Pelvis
		RHJC = {-(0.21*InterASISDist)-mm,
				 -(0.32*InterASISDist),
				 (0.34*InterASISDist)}*Pelvis
		OUTPUT(LHJC,RHJC)
	EndIf

	If $OptimalHip==1
		InterASISDist=$InterASISdist
			If $InterASISdist == 0
				InterASISDist=DIST(LASI,RASI)
			EndIf
		aa = InterASISDist/2
		LHJC=$LOptimalHip*Pelvis
		RHJC=$ROptimalHip*Pelvis
		OUTPUT(LHJC,RHJC) 
	EndIf
	DrawSegment(Pelvis)


{* =============================================================================================	*}
{* 								  Trunk, Thorax & Torso								    			*}
{* =============================================================================================	*}
{* use this replace macro if there are missing thorax or torso markers, which cannot be interpolated 	*}
{* USE WISELY!!	*}
{*SUBSTITUTE4(STRN,CLAV,C7,T10)*}
{*SUBSTITUTE4(STRN,C7,RACR,T10)*}
{*SUBSTITUTE4(CLAV,C7,RACR,T10)*}
{*SUBSTITUTE4(RACR,CLAV,C7,T10)*}
{*SUBSTITUTE4(RACR,CLAV,C7,STRN)*}

{*===================================Virtual Thorax===================================*}

{*Creates dummy thorax*}

	If ExistAtAll (VTR1,VTR2,VTR3)
		VirtualThoraxOrigin=(VTR1+VTR2+VTR3)/3	
	ELSIF ExistAtAll (VTR2,VTR3,VTR4,$VTR1RelTech)
		VirtualThoraxOrigin2=(VTR2+VTR3+VTR4)/3
		DummyThorax2=[VirtualThoraxOrigin2,VTR2-VTR4,VirtualThoraxOrigin2-VTR3,yxz]
		VTR1=$VTR1RelTech*DummyThorax2
		OUTPUT(VTR1)
		VirtualThoraxOrigin=(VTR1+VTR2+VTR3)/3	
	ELSIF ExistAtAll (VTR1,VTR3,VTR4,$VTR2RelTech)	
		VirtualThoraxOrigin3=(VTR1+VTR3+VTR4)/3
		DummyThorax3=[VirtualThoraxOrigin3,VirtualThoraxOrigin3-VTR4,VTR1-VTR3,yxz]
		VTR2=$VTR2RelTech*DummyThorax3
		OUTPUT(VTR2)
		VirtualThoraxOrigin=(VTR1+VTR2+VTR3)/3	
	ELSE ExistAtAll (VTR1,VTR2,VTR4,$VTR3RelTech)	
		VirtualThoraxOrigin4=(VTR1+VTR2+VTR4)/3
		DummyThorax4=[VirtualThoraxOrigin4,VTR2-VTR4,VTR1-VirtualThoraxOrigin4,yxz]
		VTR3=$VTR3RelTech*DummyThorax4
		OUTPUT(VTR3)
		VirtualThoraxOrigin=(VTR1+VTR2+VTR3)/3	
	EndIf

	DummyThorax=[VirtualThoraxOrigin,VTR2-VirtualThoraxOrigin,VTR1-VTR3,yxz]
	OUTPUT(VirtualThoraxOrigin)
	
{* Output pointer defined anatomical points relative to dummy segment. Reads the parameters created during the pointer trials *}

	If ExistAtAll($VT10pointerRelTech,VirtualThoraxOrigin)
		VT10 = $VT10pointerRelTech*DummyThorax
		OUTPUT(VT10)
	EndIf	
	
	If ExistAtAll($VSTRNpointerRelTech,VirtualThoraxOrigin)
		VSTRN = $VSTRNpointerRelTech*DummyThorax
		OUTPUT(VSTRN)
	EndIf		

	If ExistAtAll($VC7pointerRelTech,VirtualThoraxOrigin)
		VC7 = $VC7pointerRelTech*DummyThorax
		OUTPUT(VC7)
	EndIf	
	
{* =============================================================================================*}		
	
{*Create Thorax Segment. Allowance for virtual C7, T10 and sternum markers added by Dan Cottam, May 2016*}

	If ExistAtAll (VTR2,C7)
		ThoraxOrigin = (C7+VTR2)/2
	ELSIF ExistAtAll (VC7)
		ThoraxOrigin = (VC7+VTR2)/2
	ELSE ThoraxOrigin = (C7+CLAV)/2
	EndIf

	If ExistAtAll (VSTRN,T10)
		TorsoOrigin =(VSTRN+T10)/2
	ELSIF ExistAtAll (STRN,VT10)
		TorsoOrigin =(STRN+VT10)/2
	ELSIF ExistAtAll (VSTRN,VT10)
		TorsoOrigin =(VSTRN+VT10)/2
	ELSE TorsoOrigin =(STRN+T10)/2
	EndIf
	
	MidHJC =(LHJC+RHJC)/2
	LowerTorsoOrigin=MidHJC
	OUTPUT(ThoraxOrigin,TorsoOrigin,LowerTorsoOrigin,MidHJC)

	If ExistAtAll (VTR2,C7)
		Thorax = [ThoraxOrigin,ThoraxOrigin-TorsoOrigin,VTR2-C7,yzx]
	ELSIF ExistAtAll (VTR2,VC7)
		Thorax = [ThoraxOrigin,ThoraxOrigin-TorsoOrigin,VTR2-VC7,yzx]
	ELSE Thorax = [ThoraxOrigin,ThoraxOrigin-TorsoOrigin,CLAV-C7,yzx]
	EndIf
	
	DrawSegment(Thorax)	

{*Create Torso Segment*}		
	
	If ExistAtAll (VSTRN,T10)
		Torso = [TorsoOrigin,TorsoOrigin-LowerTorsoOrigin, VSTRN-T10, yzx]
	ElSIF ExistAtAll (STRN,VT10)
		Torso = [TorsoOrigin,TorsoOrigin-LowerTorsoOrigin, STRN-VT10, yzx]
	ElSIF ExistAtAll (VSTRN,VT10)
		Torso = [TorsoOrigin,TorsoOrigin-LowerTorsoOrigin, VSTRN-VT10, yzx]
	ELSE Torso = [TorsoOrigin,TorsoOrigin-LowerTorsoOrigin, STRN-T10, yzx]
	EndIf
	
	DrawSegment(Torso)	

{*Create Trunk Segment. A previously used definition of the trunk is saved in the UWA legacy code mod file. Trunk segment added by Dan Cottam, May 2016*}

	If ExistAtAll (VC7)
		Trunk = [MidHJC,RHJC-LHJC,VC7-MidHJC,zxy]
	ELSE Trunk = [MidHJC,RHJC-LHJC,C7-MidHJC,zxy]
	EndIf
	
	DrawSegment (Trunk)	
	
{* Create Virtual C7, MidHip Joint Centre and Sternum points relative to respective Segments *}
{*
	CreateRelPoint(C7,Thorax,C7relThorax)
	OUTPUT(C7relThorax)

	CreateRelPoint(MidHJC,TorsoAndLowerTorso,MidHJCrelTorsoAndLowerTorso)
	
	CreateRelPoint(STRN,TorsoAndLowerTorso,STRNrelTorsoAndLowerTorso)

	{* Create Omphalion *}
	{* Torso percentages were created from data in de Leva Table 1 *****}

	TrunkLength=DIST(MidHJCrelTorsoAndLowerTorso,C7relThorax) 
	IF $Female==0
		OMPHDist = .3038 * TrunkLength
		OMPHPoint = OMPHDist * {0,0,1}
		OMPH=STRNrelTorsoAndLowerTorso-OMPHPoint
		OUTPUT(OMPH)
	EndIF

	IF $Female==1
		OMPHDist = .2975 * TrunkLength
		OMPHPoint = OMPHDist * {0,0,1}
		OMPH=STRNrelTorsoAndLowerTorso-OMPHPoint
		OUTPUT(OMPH)
	EndIF

	{********** Create Middle and Lower Torso Segments *********}

	Torso = [TorsoOrigin,TorsoOrigin-OMPH, STRN-T10, yzx]
	DrawSegment(Torso)

	LowerTorso = [LowerTorsoOrigin,OMPH-LowerTorsoOrigin, LASI-RASI, yxz]
	DrawSegment(LowerTorso)
*}

{* =====================================================================================*}
{*==================================Head================================================*}
{* =====================================================================================*}
{* use this replace macro if there are missing head markers, which cannot be interpolated *}
{* SUBSTITUTE4(LFHD, LBHD, RFHD, RBHD)*}


	HeadOrigin = (LFHD+RFHD)/2
	LeftSideHead = (LFHD+LBHD)/2
	RightSideHead = (RFHD+RBHD)/2
	MidBackHead = (LBHD+RBHD)/2
	HeadMid = (LFHD+RFHD+LBHD+RBHD)/4

	HeadSize = DIST(HeadOrigin,MidBackHead)

	CHead = MidBackHead+$MarkerDiameter*(HeadOrigin-MidBackHead)/(2*HeadSize)
	Head = [CHead,RightSideHead-LeftSideHead,MidBackHead-HeadOrigin,zyx]
	
	{*
	TopHead=$TopHeadRelTech*Head
	HeadLength = Dist(C7relThorax,TopHead)
	HeadCOMGlob=(HeadLength*HeadPos)*Thorax

	OUTPUT(HeadCOMGlob,TopHead,HeadOrigin,HeadMid)
	DrawSegment(Head)
	*}

{*========================================================================================*}
{*						Start Upper Limb Static Section				  					  *}
{*========================================================================================*}

{* =====================================================================================*}
{* 									Acromion								    		*}
{* =====================================================================================*}

	If EXISTATALL(LACR1,LACR3)
		LACR = (LACR1+LACR3)/2
	EndIf
	OUTPUT(LACR)

	If EXISTATALL(RACR1,RACR3)
		RACR = (RACR1+RACR3)/2
	EndIf
	OUTPUT(RACR)

	RAcromionOrigin = (RACR1+RACR3)/2
	LAcromionOrigin = (LACR1+LACR3)/2
	RAcromion = [RAcromionOrigin,RACR3-RACR1,RACR-RACR2,xzy]
	LAcromion = [LAcromionOrigin,LACR3-LACR1,LACR-LACR2,xzy]
	DrawSegment(RAcromion)
	DrawSegment(LAcromion)
	OUTPUT(RAcromionOrigin,LAcromionOrigin)
	
{* =====================================================================================*}
{* 					Proximal upper arm			    									*}
{* =====================================================================================*}
{*Fourth marker added May 2016*}

{*Right Proximal Upper Arm*}

	If ExistAtAll (RUA1,RUA2,RUA3)
		pRightUpperArmOrigin=(RUA1+RUA2+RUA3)/3
	ELSIF ExistAtAll(RUA2,RUA3,RUA4,$RUA1RelTech)
		pRightUpperArmOrigin2=(RUA2+RUA3+RUA4)/3
		pRightUpperArm2 = [pRightUpperArmOrigin2,pRightUpperArmOrigin2-RUA3,RUA2-RUA4,yzx]
		RUA1=$RUA1RelTech*pRightUpperArm2
		OUTPUT(RUA1)
		pRightUpperArmOrigin=(RUA1+RUA2+RUA3)/3
	ELSIF ExistAtAll(RUA1,RUA3,RUA4,$RUA2RelTech)
		pRightUpperArmOrigin3=(RUA1+RUA3+RUA4)/3
		pRightUpperArm3 = [pRightUpperArmOrigin3,RUA1-RUA3,RUA4-pRightUpperArmOrigin3,yzx]
		RUA2=$RUA2RelTech*pRightUpperArm3
		OUTPUT(RUA2)
		pRightUpperArmOrigin=(RUA1+RUA2+RUA3)/3
	ELSE ExistAtAll(RUA1,RUA2,RUA4,$RUA3RelTech)
		pRightUpperArmOrigin4=(RUA1+RUA2+RUA4)/3
		pRightUpperArm4 = [pRightUpperArmOrigin4,RUA1-pRightUpperArmOrigin4,RUA2-RUA4,yzx]
		RUA3=$RUA3RelTech*pRightUpperArm4
		OUTPUT(RUA3)
		pRightUpperArmOrigin=(RUA1+RUA2+RUA3)/3
	EndIf

	pRightUpperArm = [pRightUpperArmOrigin,RUA1-RUA3,RUA2-pRightUpperArmOrigin,yzx] 
	OUTPUT(pRightUpperArmOrigin)
	
	DrawSegment(pRightUpperArm)

{*Left Proximal Upper Arm*}
	
	If ExistAtAll (LUA1,LUA2,LUA3)
		pLeftUpperArmOrigin=(LUA1+LUA2+LUA3)/3
	ELSIF ExistAtAll(LUA2,LUA3,LUA4,$LUA1RelTech)
		pLeftUpperArmOrigin2=(LUA2+LUA3+LUA4)/3
		pLeftUpperArm2 = [pLeftUpperArmOrigin2,pLeftUpperArmOrigin2-LUA3,LUA2-LUA4,yzx]
		LUA1=$LUA1RelTech*pLeftUpperArm2
		OUTPUT(LUA1)
		pLeftUpperArmOrigin=(LUA1+LUA2+LUA3)/3
	ELSIF ExistAtAll(LUA1,LUA3,LUA4,$LUA2RelTech)
		pLeftUpperArmOrigin3=(LUA1+LUA3+LUA4)/3
		pLeftUpperArm3 = [pLeftUpperArmOrigin3,LUA1-LUA3,LUA4-pLeftUpperArmOrigin3,yzx]
		LUA2=$LUA2RelTech*pLeftUpperArm3
		OUTPUT(LUA2)
		pLeftUpperArmOrigin=(LUA1+LUA2+LUA3)/3
	ELSE ExistAtAll(LUA1,LUA2,LUA4,$LUA3RelTech)
		pLeftUpperArmOrigin4=(LUA1+LUA2+LUA4)/3
		pLeftUpperArm4 = [pLeftUpperArmOrigin4,LUA1-pLeftUpperArmOrigin4,LUA2-LUA4,yzx]
		LUA3=$LUA3RelTech*pLeftUpperArm4
		OUTPUT(LUA3)
		pLeftUpperArmOrigin=(LUA1+LUA2+LUA3)/3
	EndIf

	pLeftUpperArm = [pLeftUpperArmOrigin,LUA1-LUA3,LUA2-pLeftUpperArmOrigin,yzx] 
	OUTPUT(pLeftUpperArmOrigin)
	
	DrawSegment(pLeftUpperArm)
	
{* =============================================================================================*}
{* 	Shoulder Joint,"Shoulder Wings" & Single Shoulder Segment - "ShldrAlign" 		*}
{* =============================================================================================*}
{*SUBSTITUTE4(LUA1,LUA2,LUA3,LACR)*}
{*SUBSTITUTE4(RUA1,RUA2,RUA3,RACR)*}

{*Define Virtual Shoulder joint centres in the Global Coordinate System*}
	
	LSJCa = $LSJCA*LAcromion
	LSJCb = $LSJCB*pLeftUpperArm
	LSJC = (LSJCa+LSJCb)/2
	OUTPUT(LSJC)
	RSJCa = $RSJCA*RAcromion
	RSJCb = $RSJCB*pRightUpperArm
	RSJC = (RSJCa+RSJCb)/2
	OUTPUT(RSJC)


{* Create a Shoulder "Wing" Segment, which is a plane created from midpoint between C7 and CLAV (ThoraxOrigin) *} 
{* Shoulder marker (xACR) and modelled shoulder joint centre (xSJC) *}
	LeftShoulderWing = [LSJC, ThoraxOrigin-LSJC, LACR-LSJC, zxy]
	RightShoulderWing = [RSJC, RSJC-ThoraxOrigin, RACR-RSJC, zxy]
	
{* Create a single shoulder segment to allow calculation of hip-shoulder separation angle *}
{* and comparison to previous trunk research (e.g. fast bowling back injury work*} 

	If ExistatAll(LSJC, RSJC)
		MidShldr = (LSJC+RSJC)/2
		ShldrAlign = [MidShldr,RSJC-LSJC,C7-ThoraxOrigin,zyx]
	Else
		MidShldr = (LACR+RACR)/2
		ShldrAlign = [MidShldr,RACR-LACR,C7-ThoraxOrigin,zyx]
	EndIf
		
	OUTPUT(MidShldr)
	DrawSegment(LeftShoulderWing)
	DrawSegment(RightShoulderWing)
	DrawSegment(ShldrAlign)


{*======================================================================================*}
{*   						Distal Upper Arm 										*}
{*======================================================================================*}
{* use this replace macro if there are missing upper arm triad markers, which cannot be interpolated *}
{*SUBSTITUTE4(LUA1, LUA2, LACR, LUA3)*} 
{*SUBSTITUTE4(RUA1, RUA2, RACR, RUA3)*}

{*Fourth marker added May 2016*}

{*Right Distal Upper Arm*}

	If ExistAtAll (dRUA1,dRUA2,dRUA3)
		dRightUpperArmOrigin=(dRUA1+dRUA2+dRUA3)/3
	ELSIF ExistAtAll(dRUA2,dRUA3,dRUA4,$dRUA1RelTech)
		dRightUpperArmOrigin2=(dRUA2+dRUA3+dRUA4)/3
		dRightUpperArm2 = [dRightUpperArmOrigin2,dRightUpperArmOrigin2-dRUA3,dRUA2-dRUA4,zxy]
		dRUA1=$dRUA1RelTech*dRightUpperArm2
		OUTPUT(dRUA1)
		dRightUpperArmOrigin=(dRUA1+dRUA2+dRUA3)/3
	ELSIF ExistAtAll(dRUA1,dRUA3,dRUA4,$dRUA2RelTech)
		dRightUpperArmOrigin3=(dRUA1+dRUA3+dRUA4)/3
		dRightUpperArm3 = [dRightUpperArmOrigin3,dRUA1-dRUA3,dRUA4-dRightUpperArmOrigin3,zxy]
		dRUA2=$dRUA2RelTech*dRightUpperArm3
		OUTPUT(dRUA2)
		dRightUpperArmOrigin=(dRUA1+dRUA2+dRUA3)/3
	ELSE ExistAtAll(dRUA1,dRUA2,dRUA4,$dRUA3RelTech)
		dRightUpperArmOrigin4=(dRUA1+dRUA2+dRUA4)/3
		dRightUpperArm4 = [dRightUpperArmOrigin4,dRUA1-dRightUpperArmOrigin4,dRUA2-dRUA4,zxy]
		dRUA3=$dRUA3RelTech*dRightUpperArm4
		OUTPUT(dRUA3)
		dRightUpperArmOrigin=(dRUA1+dRUA2+dRUA3)/3
	EndIf

	dRightUpperArm = [dRightUpperArmOrigin,dRUA1-dRUA3,dRUA2-dRightUpperArmOrigin,zxy] 
	OUTPUT(dRightUpperArmOrigin)
	
	DrawSegment(dRightUpperArm)

{*Left Distal Upper Arm*}
	
	If ExistAtAll (dLUA1,dLUA2,dLUA3)
		dLeftUpperArmOrigin=(dLUA1+dLUA2+dLUA3)/3
	ELSIF ExistAtAll(dLUA2,dLUA3,dLUA4,$dLUA1RelTech)
		dLeftUpperArmOrigin2=(dLUA2+dLUA3+dLUA4)/3
		dLeftUpperArm2 = [dLeftUpperArmOrigin2,dLeftUpperArmOrigin2-dLUA3,dLUA2-dLUA4,zxy]
		dLUA1=$dLUA1RelTech*dLeftUpperArm2
		OUTPUT(dLUA1)
		dLeftUpperArmOrigin=(dLUA1+dLUA2+dLUA3)/3
	ELSIF ExistAtAll(dLUA1,dLUA3,dLUA4,$dLUA2RelTech)
		dLeftUpperArmOrigin3=(dLUA1+dLUA3+dLUA4)/3
		dLeftUpperArm3 = [dLeftUpperArmOrigin3,dLUA1-dLUA3,dLUA4-dLeftUpperArmOrigin3,zxy]
		dLUA2=$dLUA2RelTech*dLeftUpperArm3
		OUTPUT(dLUA2)
		dLeftUpperArmOrigin=(dLUA1+dLUA2+dLUA3)/3
	ELSE ExistAtAll(dLUA1,dLUA2,dLUA4,$dLUA3RelTech)
		dLeftUpperArmOrigin4=(dLUA1+dLUA2+dLUA4)/3
		dLeftUpperArm4 = [dLeftUpperArmOrigin4,dLUA1-dLeftUpperArmOrigin4,dLUA2-dLUA4,zxy]
		dLUA3=$dLUA3RelTech*dLeftUpperArm4
		OUTPUT(dLUA3)
		dLeftUpperArmOrigin=(dLUA1+dLUA2+dLUA3)/3
	EndIf

	dLeftUpperArm = [dLeftUpperArmOrigin,dLUA1-dLUA3,dLUA2-dLeftUpperArmOrigin,zxy] 
	OUTPUT(dLeftUpperArmOrigin)
	
	DrawSegment(dLeftUpperArm)

{* Recreates epicondlye marker positions initially defined in the static trials*}

{* Pointer Output - Pointer-defined epicondyle marker positions initially defined in the static trials*}

If $Pointer==1

		RiMEL= $RMELpointerRelTech*dRightUpperArm
		OUTPUT(RiMEL)
		
		RiLEL= $RLELpointerRelTech*dRightUpperArm
		OUTPUT(RiLEL)

		LeMEL=$LMELpointerRelTech*dLeftUpperArm
		OUTPUT(LeMEL)			

		LeLEL=$LLELpointerRelTech*dLeftUpperArm
		OUTPUT(LeLEL)
EndIf

{*Marker Output - Outputs anat epicondyle marker positions initially defined in the static trials*}
	
If $Pointer==0	


		RiMEL= $RiMEL*dRightUpperArm
		OUTPUT(RiMEL)
		
		RiLEL= $RiLEL*dRightUpperArm
		OUTPUT(RiLEL)

		LeMEL=$LeMEL*dLeftUpperArm
		OUTPUT(LeMEL)			

		LeLEL=$LeLEL*dLeftUpperArm
		OUTPUT(LeLEL)
		
EndIf

LMidEp=(LeLEL+LeMEL)/2
RMidEp=(RiLEL+RiMEL)/2

LEJC=LMidEp
REJC=RMidEp

OUTPUT(LEJC,REJC)


{************************** UpperArm Anatomical CS Section  ********************}

{* Upper limb segment with Y (IR/ER) long axis defined first (pointer defined epicondyle positions)*}

	If ExistatAll(LEJC,LSJC)
		AnatLeftUpperArmY=[LEJC,LSJC-LEJC,LeLEL-LeMEL,yxz]
		DrawSegment(AnatLeftUpperArmY)
	EndIf
	 
	If ExistatAll(REJC,RSJC)
		AnatRightUpperArmY=[REJC,RSJC-REJC,RiMEL-RiLEL,yxz]
		DrawSegment(AnatRightUpperArmY)
	EndIf

{* Upper limb segment with Z(F/E) axis defined first (pointer defined epicondyle positions)*}

	If ExistatAll(LEJC,LSJC)
		AnatLeftUpperArmZ=[LEJC,LeMEL-LeLEL,LSJC-LEJC,zxy]
		DrawSegment(AnatLeftUpperArmZ)
	EndIf
	 
	If ExistatAll(REJC,RSJC)
		AnatRightUpperArmZ=[REJC,RiLEL-RiMEL,RSJC-REJC,zxy]
		DrawSegment(AnatRightUpperArmZ)
	EndIf


{***************************** Helical Section ************************************************}

{*======================================================================================*}
{*     Define Elbow Joint Centres     *}
{*======================================================================================*}

	If $HelicalElbow==1
	{* Stores SCoRE/SARA calculated EJC + FE axis of rotation for use with dynamic UWA code, added by D Wells April 2018 *}

		LEHel1=$LEHA1*dLeftUpperArm
		LEHel2=$LEHA2*dLeftUpperArm  
		REHel1=$REHA1*dRightUpperArm
		REHel2=$REHA2*dRightUpperArm
		OUTPUT(LEHel1,LEHel2,REHel1,REHel2)
		
		feLEJC=PERP(LMidEp,LEHel1,LEHel2)
		feREJC=PERP(RMidEp,REHel1,REHel2)
		
		OUTPUT(feLEJC,feREJC)
	
		{* check to make sure the helical axes vectors are in the right direction *}
		
		check1=COMP(LEHel2-LEHel1,LeMEL-LeLEL)
		check2=COMP(REHel2-REHel1,RiLEL-RiMEL)
			If check1 < 0
				hLeftUpperArm1=[feLEJC,LSJC-feLEJC,LEHel2-LEHel1,yxz]
				hLeftUpperArm2=[feLEJC,LEHel1-LEHel2,LSJC-feLEJC,zxy]
			EndIf
			If check2 < 0
				hRightUpperArm1=[feREJC,RSJC-feREJC,REHel2-REHel1,yxz]
				hRightUpperArm2=[feREJC,REHel1-REHel2,RSJC-feREJC,zxy]
			EndIf
			If check1 > 0
				hLeftUpperArm1=[feLEJC,LSJC-feLEJC,LEHel1-LEHel2,yxz]
				hLeftUpperArm2=[feLEJC,LEHel2-LEHel1,LSJC-feLEJC,zxy]
			EndIf
			If check2 > 0 
				hRightUpperArm1=[feREJC,RSJC-feREJC,REHel1-REHel2,yxz]
				hRightUpperArm2=[feREJC,REHel2-REHel1,RSJC-feREJC,zxy]
			EndIf
	EndIf

{* Define the upper limb segment using helical axis points with Y (IR/ER) long axis first*}

	If ExistatAll(feLEJC,LSJC)
		HelLeftUpperArmY=hLeftUpperArm1
		DrawSegment(HelLeftUpperArmY)
	EndIf

	If ExistatAll(feREJC,RSJC)
		HelRightUpperArmY=hRightUpperArm1
		DrawSegment(HelRightUpperArmY)
	EndIf

{* Define the upper limb segment using helical axis points with Z (F/E) axis first*}

	If ExistatAll(feLEJC,LSJC)
		HelLeftUpperArmZ=hLeftUpperArm2
		DrawSegment(HelLeftUpperArmZ)
	EndIf

	If ExistatAll(feREJC,RSJC)
		HelRightUpperArmZ=hRightUpperArm2
		DrawSegment(HelRightUpperArmZ)
	EndIf

{*==================================================================================*}
{* 					Forearm    				    *}
{*==================================================================================*}
{* 
only use this replace macro if desperate and you have missing forearm triad markers!
SUBSTITUTE4(LFA1,LFA2,LEJC,LFA3) 
SUBSTITUTE4(RFA1,RFA2,REJC,RFA3) 
SUBSTITUTE4(RFA1,RFA2,pREJC,RFA3)
SUBSTITUTE4(LFA1,LFA2,pLEJC,LFA3)
*}

{*Fourth marker added May 2016*}

{*create dummy right forearm segment*}

	If ExistAtAll (RFA1,RFA2,RFA3)
		RightForearmOrigin=(RFA1+RFA2+RFA3)/3
	ELSIF ExistAtAll(RFA2,RFA3,RFA4,$RFA1RelTech)
		RightForearmOrigin2=(RFA2+RFA3+RFA4)/3
		DummyRightForearm2 = [RightForearmOrigin2,RightForearmOrigin2-RFA3,RFA2-RFA4,zxy]
		RFA1=$RFA1RelTech*DummyRightForearm2
		OUTPUT(RFA1)
		RightForearmOrigin=(RFA1+RFA2+RFA3)/3
	ELSIF ExistAtAll(RFA1,RFA3,RFA4,$RFA2RelTech)
		RightForearmOrigin3=(RFA1+RFA3+RFA4)/3
		DummyRightForearm3 = [RightForearmOrigin3,RFA1-RFA3,RFA4-RightForearmOrigin3,zxy]
		RFA2=$RFA2RelTech*DummyRightForearm3
		OUTPUT(RFA2)
		RightForearmOrigin=(RFA1+RFA2+RFA3)/3
	ELSE ExistAtAll(RFA1,RFA2,RFA4,$RFA3RelTech)
		RightForearmOrigin4=(RFA1+RFA2+RFA4)/3
		DummyRightForearm4 = [RightForearmOrigin4,RFA1-RightForearmOrigin4,RFA2-RFA4,zxy]
		RFA3=$RFA3RelTech*DummyRightForearm4
		OUTPUT(RFA3)
		RightForearmOrigin=(RFA1+RFA2+RFA3)/3
	EndIf

	DummyRightForearm = [RightForearmOrigin,RFA1-RFA3,RFA2-RightForearmOrigin,zxy]  
	OUTPUT(RightForearmOrigin)
	
{*create dummy left forearm segment*}
	
	If ExistAtAll(LFA1,LFA2,LFA3)
		LeftForearmOrigin=(LFA1+LFA2+LFA3)/3
	ELSIF ExistAtAll(LFA2,LFA3,LFA4,$LFA1RelTech)
		LeftForearmOrigin2=(LFA2+LFA3+LFA4)/3
		DummyLeftForearm2 = [LeftForearmOrigin2,LeftForearmOrigin2-LFA3,LFA2-LFA4,zxy]
		LFA1=$LFA1RelTech*DummyLeftForearm2
		OUTPUT(LFA1)
		LeftForearmOrigin=(LFA1+LFA2+LFA3)/3
	ELSIF ExistAtAll(LFA1,LFA3,LFA4,$LFA2RelTech)
		LeftForearmOrigin3=(LFA1+LFA3+LFA4)/3
		DummyLeftForearm3 = [LeftForearmOrigin3,LFA1-LFA3,LFA4-LeftForearmOrigin3,zxy]
		LFA2=$LFA2RelTech*DummyLeftForearm3
		OUTPUT(LFA2)
		LeftForearmOrigin=(LFA1+LFA2+LFA3)/3
	ELSE ExistAtAll(LFA1,LFA2,LFA4,$LFA3RelTech)
		LeftForearmOrigin4=(LFA1+LFA2+LFA4)/3
		DummyLeftForearm4 = [LeftForearmOrigin4,LFA1-LeftForearmOrigin4,LFA2-LFA4,zxy]
		LFA3=$LFA3RelTech*DummyLeftForearm4
		OUTPUT(LFA3)
		LeftForearmOrigin=(LFA1+LFA2+LFA3)/3
	EndIf
	
	DummyLeftForearm = [LeftForearmOrigin,LFA1-LFA3,LFA2-LeftForearmOrigin,zxy]  
	OUTPUT(LeftForearmOrigin)

{***************************** Wrist Joint Centre Section ************************************************}

{* define virtual anatomical wrist landmarks relative to dummy forearm segment cluster in the Global Coord System. These will only be output if physical markers are not present. *}
	
	If ExistatAll(LWRR,LWRU)
		LWJC = (LWRR+LWRU)/2
		OUTPUT(LWJC)
	ELSE
		LWRR=$LWRRAnatRelTech*DummyLeftForearm
		LWRU=$LWRUAnatRelTech*DummyLeftForearm
		LWJC = (LWRR+LWRU)/2
		OUTPUT(LWRR,LWRU,LWJC)
	EndIf

	If ExistatAll(RWRR,RWRU)
		RWJC = (RWRR+RWRU)/2
		OUTPUT(RWJC)
	ELSE 
		RWRR=$RWRRAnatRelTech*DummyRightForearm
		RWRU=$RWRUAnatRelTech*DummyRightForearm
		RWJC = (RWRR+RWRU)/2
		OUTPUT(RWRR,RWRU,RWJC)
	EndIf

{************************************** Forearm Anatomical CS Section ****************************************}

{*************** 3-DOF **********************}

{* Define the forearm segments using the wrist axes and pointer defined EJC with Y (IR/ER) axis defined first*}

	Anat3DOFLeftForearmY=[LWJC,LEJC-LWJC,LWRR-LWRU,yxz]
	DrawSegment(Anat3DOFLeftForearmY)

	Anat3DOFRightForearmY=[RWJC,REJC-RWJC,RWRU-RWRR,yxz]
	DrawSegment(Anat3DOFRightForearmY)

{* Define the forearm segments using the wrist axes and pointer defined EJC with Z (F/E) axis defined first*}

	Anat3DOFLeftForearmZ=[LWJC,LWRR-LWRU,LEJC-LWJC,zxy]
	DrawSegment(Anat3DOFLeftForearmZ)

	Anat3DOFRightForearmZ=[RWJC,RWRU-RWRR,REJC-RWJC,zxy]
	DrawSegment(Anat3DOFRightForearmZ)


{***************** 2-DOF ***************************}

{* Define the forearm segments using elbow axes and pointer defined EJC with Y (IR/ER) axis defined first*}

	Anat2DOFLeftForearmY=[LWJC,LEJC-LWJC,LeMEL-LeLEL,yxz]
	DrawSegment(Anat2DOFLeftForearmY)

	Anat2DOFRightForearmY=[RWJC,REJC-RWJC,RiLEL-RiMEL,yxz]
	DrawSegment(Anat2DOFRightForearmY)

{* Define the forearm segments using elbow axes and pointer defined EJC with Z (F/E) axis defined first*}

	Anat2DOFLeftForearmZ=[LWJC,LeMEL-LeLEL,LEJC-LWJC,zxy]
	DrawSegment(Anat2DOFLeftForearmZ)

	Anat2DOFRightForearmZ=[RWJC,RiLEL-RiMEL,REJC-RWJC,zxy]
	DrawSegment(Anat2DOFRightForearmZ)

{******************************************* Helical Section ************************************************}

{* Based on the helical axis of supination/pronation in fixed flexion; also determine sup/pro EJC *}
	
	If $HelicalElbow == 1
		LEspHel1=$LEspHA1*DummyLeftForearm
		LEspHel2=$LEspHA2*DummyLeftForearm
		REspHel1=$REspHA1*DummyRightForearm
		REspHel2=$REspHA2*DummyRightForearm
		OUTPUT(LEspHel1,LEspHel2,REspHel1,REspHel2)

		spLWJC=PERP(LWJC,LEspHel1,LEspHel2)
		spRWJC=PERP(RWJC,REspHel1,REspHel2)

		OUTPUT(spLWJC,spRWJC)
	EndIf

{************************** Forearm Anatomical CS Section (Using Helical) ********************}

{*************** 3-DOF **********************}

{* Define the forearm segment using helical axis points with Y (IR/ER) axis defined first*}

	Hel3DOFLeftForearmY=[LWJC,LEspHel2-LEspHel1,LWRR-LWRU,yxz]
	DrawSegment(Hel3DOFLeftForearmY)

	Hel3DOFRightForearmY=[RWJC,REspHel2-REspHel1,RWRU-RWRR,yxz]
	DrawSegment(Hel3DOFRightForearmY)

{* Define the forearm segment using helical axis points with Z (F/E) axis defined first*}

	Hel3DOFLeftForearmZ=[LWJC,LWRU-LWRR,LEspHel2-LEspHel1,zxy]
	DrawSegment(Hel3DOFLeftForearmZ)

	Hel3DOFRightForearmZ=[RWJC,RWRR-RWRU,REspHel2-REspHel1,zxy]
	DrawSegment(Hel3DOFRightForearmZ)


{***************** 2-DOF ***************************}

{* Define the forearm segment using helical axis points with Y (IR/ER) long axis first*}
	
	If ExistatAll(spLWJC,LEJC)
		Hel2DOFLeftForearmY1=[spLWJC,LEspHel1-LEspHel2,LEHel1-LEHel2,yxz]
		DrawSegment(Hel2DOFLeftForearmY1)

		Hel2DOFLeftForearmY2=[spLWJC,LEspHel2-LEspHel1,LWRR-LWRU,yxz]
		DrawSegment(Hel2DOFLeftForearmY2)
	EndIf

	If ExistatAll(spRWJC,REJC)
		Hel2DOFRightForearmY2=[spRWJC,REspHel2-REspHel1,REHel1-REHel2,yxz]
		DrawSegment(Hel2DOFRightForearmY2)

		Hel2DOFRightForearmY1=[spRWJC,REspHel2-REspHel1,RWRU-RWRR,yxz]
		DrawSegment(Hel2DOFRightForearmY1)
	EndIf

{* Define the forearm segment using helical axis points with Z (F/E) axis first*}

	If ExistatAll(spLWJC,feLEJC)
		Hel2DOFLeftForearmZ=[spLWJC,LEHel1-LEHel2,feLEJC-spLWJC,zxy]
		DrawSegment(Hel2DOFLeftForearmZ)
	EndIf

	If ExistatAll(spRWJC,feREJC)
		Hel2DOFRightForearmZ2=[spRWJC,REHel1-REHel2,feREJC-spRWJC,zxy]
		DrawSegment(Hel2DOFRightForearmZ2)
	EndIf

	If ExistatAll(RWJC,feREJC)
		Hel2DOFRightForearmZ=[RWJC,REHel1-REHel2,feREJC-RWJC,zxy]
		DrawSegment(Hel2DOFRightForearmZ)
	EndIf

{*======================================================================================*}
{* 					Hand               				*}
{*======================================================================================*}
{* only use this SUBSTITUTE macro if desperate and you have missing HAND markers!*}
{*SUBSTITUTE4(LHNR,LHNU,LCAR,LWJC)*}
{*SUBSTITUTE4(RHNR,RHNU,RCAR,RWJC)*}

	If EXISTATALL(LCAR,RCAR)
		LeftHandOrigin =(LHNR+LHNU+LCAR)/3
		RightHandOrigin = (RHNR+RHNU+RCAR)/3
	ELSE
		LeftHandOrigin =(LHNR+LHNU)/2
		RightHandOrigin = (RHNR+RHNU)/2
	ENDIF

	LeftHand= [LeftHandOrigin,LWJC-LeftHandOrigin,LHNR-LHNU,yxz]
	RightHand= [RightHandOrigin, RWJC-RightHandOrigin, RHNU-RHNR, yxz]


	LHandLength=$LHandLength
		If $LHandLength == 0
			LHandLength=DIST(LCAR,LWJC)
		EndIf 

	RHandLength=$RHandLength
		If $RHandLength == 0
			RHandLength=DIST(RCAR,RWJC)
		EndIf 


	OUTPUT(LeftHandOrigin,RightHandOrigin)
	DrawSegment(LeftHand)
	DrawSegment(RightHand)

{*======================================================================================*}
{* 					Ball              				*}
{*======================================================================================*}
SUBSTITUTE4(Ball1,Ball2,Ball3,Ball4)
{*
	If EXISTATALL(Ball4)
		BallOrigin =(Ball1+Ball2+Ball3+Ball4)/4
	ELSE
		vBall = (Ball1+Ball2)/2
		BallOrigin =(vBall+Ball3)/2
	ENDIF

	Ball = [BallOrigin,Ball1-BallOrigin,Ball2-Ball1,yxz]

	DrawSegment(Ball)
	OUTPUT(BallOrigin)
	*}


	{*ICC Fast Bowling Ball Origin*}

	ICCBallOrigin = (Ball1+Ball2)/2
	OUTPUT(ICCBallOrigin)

{*Ball to Hand Distance*}	
	
	IF $HandDom == 1 
		BallPosInHand = DIST(RCAR,ICCBallOrigin)
		Holding = (BallPosInHand < $BallCarDist)
	ELSE
		BallPosInHand = DIST(LCAR,ICCBallOrigin)
		Holding= (BallPosInHand < $BallCarDist)
	ENDIF

	OUTPUT(Holding,BallPosInHand)

	IF EXISTATALL(ICCBallOrigin,RightHandOrigin)
		BallHandDist = DIST(ICCBallOrigin,RightHandOrigin)
	ENDIF

	OUTPUT(BallHandDist)
{*==================================================================*}
{*   First, find the general progression direction of the subject   *}
{*==================================================================*}

avPEL = average((PELOrigin[7] + PELOrigin[6] + PELOrigin[5] + PELOrigin[4] + PELOrigin[3]
	     - PELOrigin[-7]- PELOrigin[-6]- PELOrigin[-5]- PELOrigin[-4]- PELOrigin[-3])/5)

avTHOR = average((ThoraxOrigin[7] + ThoraxOrigin[6] + ThoraxOrigin[5] + ThoraxOrigin[4] + ThoraxOrigin[3]
	     - ThoraxOrigin[-7]- ThoraxOrigin[-6]- ThoraxOrigin[-5]- ThoraxOrigin[-4]- ThoraxOrigin[-3])/5)
OUTPUT(avPEL)
OUTPUT(avTHOR)

	IF Exist(avPEL)
		XDOT = COMP(avPEL, {1, 0, 0})
		YDOT = COMP(avPEL, {0, 1, 0})
	ELSE
		XDOT = COMP(avTHOR, {1, 0, 0})
		YDOT = COMP(avTHOR, {0, 1, 0})
	EndIf


	IF ABS(XDOT)>ABS(YDOT)
		Anatomy2ndDefLine = XDOT*{1, 0, 0}
	ELSE
		Anatomy2ndDefLine = YDOT*{0, 1, 0}
	ENDIF

IF $SubjectBasedAnatomicalCS <> 0
	Anatomy = [{0, 0, 0}, {0, 0, 1}, Anatomy2ndDefLine, yzx]
ELSE
	Anatomy = [{0, 0, 0}, {0, 0, 1}, {1, 0, 0} , yzx]
ENDIF
DrawSegment(Anatomy)



{* 				END Static Segmental Modeling 				*}


{* =====================================================================================*}
{*					Angle Calcs					*}
{*======================================================================================*}
{*Calculate joint angles, using Grood&Suntay sequence - flexion, abduction, rotation	*}
{*Head, Thorax, Torso, LowerTorso & Lumbar rotations defined in terms of the Global coordinate system*}
{*Pelvis Angle also calulated in Richard Baker's Proposed Convention - rotn, obliquity, tilt*}
{*Segment Angles are defined as parent and child segments using Euler Angles      	*}

{* ==============================================================*}
{*GLOBAL ANGLE OUTPUTS*}
{*Relative to average general progression direction of subject - now called anatomy*}
{* Assume that rotate head coordinate system to align with Anatomy Coordinate system *}

	{*
	Head = ROT(Head,1(Head),$OffsetHeadFlexExt)
	Head = ROT(Head,2(Head),$OffsetHeadTilt)
	Head = ROT(Head,3(Head),$OffsetHeadRot)
	DrawSegment(Head)
	*}

{*Head*}

	{* HeadAngle = -<Anatomy, Head, zxy> *}

{*Pelvis*}

	PelvisAngle = -<Anatomy, Pelvis, zxy>
	PelvisAngleBaker = -<Anatomy, Pelvis, yxz>

{*Trunk*}

	ThoraxAngle 	= -<Anatomy, Thorax, zxy>
	ThoraxAngleBaker = -<Anatomy, Thorax, yxz>
	TrunkAngleGlob = <Anatomy, Trunk, zxy>
	TrunkAngleBaker= <Anatomy, Trunk, yxz>
	ShldrAlignAngle = -<Anatomy, ShldrAlign,zxy>
	TorsoAngle 	= -<Anatomy, Torso, zxy>
	TorsoAngleBaker = -<Anatomy, Torso, yxz>
	LShWingAngleGlob = -<Anatomy, LeftShoulderWing, zxy>
	RShWingAngleGlob = -<Anatomy, RightShoulderWing, zxy>

{*Upper arm*}

	AnatLUpperArmAngleGlob = -<Anatomy, AnatLeftUpperArmY, zxy>
	AnatRUpperArmAngleGlob = -<Anatomy, AnatRightUpperArmY, zxy>

	HelLUpperArmAngleGlob = -<Anatomy, HelLeftUpperArmY, zxy>
	HelRUpperArmAngleGlob = -<Anatomy, HelRightUpperArmY, zxy>

{*Forearm*}

	RForearmAngleGlob = -<Anatomy,Anat3DOFRightForearmY>
	LForearmAngleGlob = -<Anatomy,Anat3DOFLeftForearmY>

{*Hand*}

	RHandAngleGlob = -<Anatomy,RightHand,zxy>
	LHandAngleGlob = -<Anatomy,LeftHand,zxy>


{******************************************************}


{* ==============================================================*}
{*RELATIVE ANGLE OUTPUTS*}
{* ==============================================================*}

{* Trunk Outputs 	*}

	TorsotoPelAngle = -<Pelvis,Torso,zxy>
	ThortoPelAngle = -<Pelvis,Thorax,zxy>
	LUpperArmtoPelAngle = -<Pelvis,AnatLeftUpperArmY,zxy>
	RUpperArmtoPelAngle = -<Pelvis,AnatRightUpperArmY,zxy>

	TorsotoThorAngle = -<Torso,Thorax,zxy>
	PelShldrSepAng = -<Pelvis,ShldrAlign,zxy>
	LShWingAngle = -<Thorax, LeftShoulderWing,zxy>
	RShWingAngle = -<Thorax, RightShoulderWing,zxy>

	{* ZXY Grood-Suntay Euler angle sequence *}
	peLShtoThoraxAngle = -<Thorax, AnatLeftUpperArmY,zxy>
	peRShtoThoraxAngle = -<Thorax, AnatRightUpperArmY,zxy>

{*Upper body Outputs*}

{*Shoulder*}

	peLShoulderAngle = -<LeftShoulderWing, AnatLeftUpperArmY,zxy>
	peRShoulderAngle = -<RightShoulderWing, AnatRightUpperArmY,zxy>

{*Elbow*}

	{* Anatomical 3DOF Angles*}
		{* Formerly YAnat3DOFLElbAng,YAnat3DOFRElbAng,ZAnat3DOFLElbAng,ZAnat3DOFRElbAng *}

		LElbow_YAnat3DOF = -<AnatLeftUpperArmY,Anat3DOFLeftForearmY,zxy>
		RElbow_YAnat3DOF = -<AnatRightUpperArmY,Anat3DOFRightForearmY,zxy>

		LElbow_ZAnat3DOF = -<AnatLeftUpperArmZ,Anat3DOFLeftForearmZ,zxy>
		RElbow_ZAnat3DOF = -<AnatRightUpperArmZ,Anat3DOFRightForearmZ,zxy>

	{*Anatomical 2DOF Angles*}
		{* Formerly FEAnat2DOFLElbAng,FEAnat2DOFRElbAng,SPAnat2DOFLElbAng,SPAnat2DOFRElbAng *}		

		LElbow_FEAnat2DOF = -<AnatLeftUpperArmZ,Anat2DOFLeftForearmZ,zxy>
		RElbow_FEAnat2DOF = -<AnatRightUpperArmZ,Anat2DOFRightForearmZ,zxy>

		LElbow_SPAnat2DOF = -<Anat2DOFLeftForearmY,Anat3DOFLeftForearmY,yxz>
		RElbow_SPAnat2DOF = -<Anat3DOFRightForearmY,Anat2DOFRightForearmY,yxz>

	{*Helical 3DOF Angles*}
		{* Formerly YHel3DOFLElbAng,YHel3DOFRElbAng,ZHel3DOFLElbAng,ZHel3DOFRElbAng *}		

		LElbow_YHel3DOF = -<HelLeftUpperArmY,Hel3DOFLeftForearmY,zxy>
		RElbow_YHel3DOF = -<HelRightUpperArmY,Hel3DOFRightForearmY,zxy>

		LElbow_ZHel3DOF = -<HelLeftUpperArmZ,Hel3DOFLeftForearmZ,zxy>
		RElbow_ZHel3DOF = -<HelRightUpperArmZ,Hel3DOFRightForearmZ,zxy>

	{*Helical 2DOF Angles*}
		{* Formerly FEHel2DOFLElbAng,FEHel2DOFRElbAng,SPHel2DOFLElbAng,SPHel2DOFRElbAng *}		

		LElbow_FEHel2DOF = -<HelLeftUpperArmZ,Hel2DOFLeftForearmZ,zxy>
		RElbow_FEHel2DOF = -<HelRightUpperArmZ,Hel2DOFRightForearmZ,zxy>

		LElbow_SPHel2DOF = -<Hel2DOFLeftForearmY2,Hel2DOFLeftForearmY1,yxz>
		RElbow_SPHel2DOF = -<Hel2DOFRightForearmY2,Hel2DOFRightForearmY1,yxz>


{*Wrist*}

	LWristAngle = -<Anat3DOFLeftForearmY, LeftHand,zxy>
	RWristAngle = -<Anat3DOFRightForearmY, RightHand,zxy>

{*
{*Relative Outputs for Tennis Racquet*}

	If $HandDom<>0
		RacqAngle = -<RightForearm, Racquet,zxy>
	Else
		RacqAngle = -<LeftForearm, Racquet,zxy>
	EndIf

	{*Relative Outputs for Hockey Stick data*}
	LStickAngle = -<LeftForearm, Stick,zxy>
	RStickAngle = -<RightForearm, Stick,zxy>

	{*Relative Outputs for the Lumbar Rig*}
	PeltoLumAngle = -<Pelvis, LumbarRegion, zxy>
	LumbarRigtoLumAngle = -<LumbarRig, LumbarRegion, zxy>
	LumtoTorsoAngle = -<LumbarRegion, Torso,zxy>
*}

{* =====================================================================================*}
{*Linear Velocity Outputs*}
{* =====================================================================================*}

	Linvelacc(MidPelvis)
	Linvelacc(RHJC)
	Linvelacc(LHJC)
	Linvelacc(ThoraxOrigin)
	Linvelacc(TorsoOrigin)
	Linvelacc(C7)
	Linvelacc(LSJC)
	Linvelacc(RSJC)
	Linvelacc(LEJC)
	Linvelacc(REJC)
	Linvelacc(LWJC)
	Linvelacc(RWJC)
	Linvelacc(ICCBallOrigin)

{* =====================================================================================*}
{*Angular Velocity Outputs*}
{* =====================================================================================*}

	ANGVELACC(Thorax,Anatomy,ThorGlob)
	ANGVELACC(Pelvis,Anatomy,PelGlob)
	ANGVELACC(Torso,Anatomy,TorsoGlob)
	{*ANGVELACC(LowerTorso,Anatomy,LTorsoGlob)*}
	ANGVELACC(HelLeftUpperArmY,Anatomy,LUppArmGlob)
	ANGVELACC(HelRightUpperArmY,Anatomy,RUppArmGlob)

	ANGVELACC(Thorax,Pelvis,ThorPel)
	ANGVELACC(Torso,Pelvis,TorsoPel)
	{*ANGVELACC(LowerTorso,Pelvis,LTorsoPel)*}

	ANGVELACC(HelLeftUpperArmY,LeftShoulderWing,LShldr)
	ANGVELACC(HelLeftUpperArmY,Thorax,LShldrThor)
	ANGVELACC(Anat3DOFLeftForearmY,HelLeftUpperArmY,LElbow)
	ANGVELACC(LeftHand,Anat3DOFLeftForearmY,LWrist)
	{*ANGVELACC(Stick,Anat3DOFLeftForearmY,LStick)*}

	ANGVELACC(HelRightUpperArmY,RightShoulderWing,RShldr)
	ANGVELACC(HelRightUpperArmY,Thorax,RShldrThor)
	ANGVELACC(Anat3DOFRightForearmY,AnatRightUpperArmY,RElbow)
	ANGVELACC(RightHand,Anat3DOFRightForearmY,RWrist)
	ANGVELACC(HelRightUpperArmY,Anatomy,RElbowGlob)

	{*
	ANGVELACC(Stick,Anatomy,Stick)
	ANGVELACC(Stick,Anat3DOFRightForearmY,RStick)
	*}

{* ---------------------------------------------- *}
{*    Putting things in right axis conventions    *}
{* ---------------------------------------------- *}	
	
	{*HeadAngle = <-1(HeadAngle),2(HeadAngle),3(HeadAngle)> *}
	
	PelvisAngle = <-1(PelvisAngle),2(PelvisAngle),3(PelvisAngle)>
	PelvisAngleBaker = <1(PelvisAngleBaker),2(PelvisAngleBaker),-3(PelvisAngleBaker)>
	ThoraxAngle = <-1(ThoraxAngle),2(ThoraxAngle),3(ThoraxAngle)>
	ThoraxAngleBaker = <1(ThoraxAngleBaker),2(ThoraxAngleBaker),-3(ThoraxAngleBaker)>
	TrunkAngleGlob = <-1(TrunkAngleGlob),2(TrunkAngleGlob),3(TrunkAngleGlob)>
	TrunkAngleBaker = <1(TrunkAngleBaker),2(TrunkAngleBaker),-3(TrunkAngleBaker)>
	TorsoAngle = <-1(TorsoAngle),2(TorsoAngle),3(TorsoAngle)>
	TorsoAngleBaker = <1(TorsoAngleBaker),2(TorsoAngleBaker),-3(TorsoAngleBaker)>
	
	LShWingAngleGlob = <1(LShWingAngleGlob),-2(LShWingAngleGlob),-3(LShWingAngleGlob)>
	RShWingAngleGlob = <1(RShWingAngleGlob),2(RShWingAngleGlob),3(RShWingAngleGlob)>
	AnatRUpperArmAngleGlob = <1(AnatRUpperArmAngleGlob),2(AnatRUpperArmAngleGlob),3(AnatRUpperArmAngleGlob)>
	AnatLUpperArmAngleGlob = <1(AnatLUpperArmAngleGlob),2(AnatLUpperArmAngleGlob),3(AnatLUpperArmAngleGlob)>
	HelRUpperArmAngleGlob = <1(HelRUpperArmAngleGlob),2(HelRUpperArmAngleGlob),3(HelRUpperArmAngleGlob)>
	HelLUpperArmAngleGlob = <1(HelLUpperArmAngleGlob),-2(HelLUpperArmAngleGlob),-3(HelLUpperArmAngleGlob)>
	RForearmAngleGlob = <1(RForearmAngleGlob),2(RForearmAngleGlob),3(RForearmAngleGlob)> 
	LForearmAngleGlob = <1(LForearmAngleGlob),-2(LForearmAngleGlob),-3(LForearmAngleGlob)>
	RHandAngleGlob = <1(RHandAngleGlob),2(RHandAngleGlob),3(RHandAngleGlob)>
	LHandAngleGlob = <1(LHandAngleGlob),-2(LHandAngleGlob),-3(LHandAngleGlob)>

	{*ShldrAlignAngle = <-1(ShldrAlignAngle),2(ShldrAlignAngle),3(ShldrAlignAngle)>
	BallAngle = <1(BallAngle),2(BallAngle),3(BallAngle)>*}

{*RELATIVE ANGLE OUTPUTS*}

	ThortoPelAngle = <-1(ThortoPelAngle),2(ThortoPelAngle),3(ThortoPelAngle)>
	TorsotoPelAngle = <-1(TorsotoPelAngle),2(TorsotoPelAngle),3(TorsotoPelAngle)>
	TorsotoThorAngle = <-1(TorsotoThorAngle),2(TorsotoThorAngle),3(TorsotoThorAngle)>
	LShWingAngle = <1(LShWingAngle),-2(LShWingAngle),-3(LShWingAngle)>
	RShWingAngle = <1(RShWingAngle),2(RShWingAngle),3(RShWingAngle)>
	peLShoulderAngle = <-1(peLShoulderAngle),-2(peLShoulderAngle),3(peLShoulderAngle)>
	peRShoulderAngle = <1(peRShoulderAngle),2(peRShoulderAngle),3(peRShoulderAngle)>

	LElbow_YAnat3DOF = <1(LElbow_YAnat3DOF),-2(LElbow_YAnat3DOF),3(LElbow_YAnat3DOF)>
	RElbow_YAnat3DOF = <1(RElbow_YAnat3DOF),2(RElbow_YAnat3DOF),3(RElbow_YAnat3DOF)>
	LElbow_ZAnat3DOF = <1(LElbow_ZAnat3DOF),-2(LElbow_ZAnat3DOF),3(LElbow_ZAnat3DOF)>
	RElbow_ZAnat3DOF = <1(RElbow_ZAnat3DOF),2(RElbow_ZAnat3DOF),3(RElbow_ZAnat3DOF)>
	LElbow_FEAnat2DOF = <1(LElbow_FEAnat2DOF),-2(LElbow_FEAnat2DOF),3(LElbow_FEAnat2DOF)>
	RElbow_FEAnat2DOF = <1(RElbow_FEAnat2DOF),2(RElbow_FEAnat2DOF),3(RElbow_FEAnat2DOF)>
	LElbow_SPAnat2DOF = <1(LElbow_SPAnat2DOF),-2(LElbow_SPAnat2DOF),-3(LElbow_SPAnat2DOF)>
	RElbow_SPAnat2DOF = <1(RElbow_SPAnat2DOF),2(RElbow_SPAnat2DOF),3(RElbow_SPAnat2DOF)>

	LElbow_YHel3DOF = <1(LElbow_YHel3DOF),-2(LElbow_YHel3DOF),-3(LElbow_YHel3DOF)>
	RElbow_YHel3DOF = <1(RElbow_YHel3DOF),2(RElbow_YHel3DOF),3(RElbow_YHel3DOF)>
	LElbow_ZHel3DOF = <1(LElbow_ZHel3DOF),-2(LElbow_ZHel3DOF),-3(LElbow_ZHel3DOF)>
	RElbow_ZHel3DOF = <1(RElbow_ZHel3DOF),2(RElbow_ZHel3DOF),3(RElbow_ZHel3DOF)>
	LElbow_FEHel2DOF = <1(LElbow_FEHel2DOF),-2(LElbow_FEHel2DOF),-3(LElbow_FEHel2DOF)>
	RElbow_FEHel2DOF = <1(RElbow_FEHel2DOF),2(RElbow_FEHel2DOF),3(RElbow_FEHel2DOF)>
	LElbow_SPHel2DOF = <1(LElbow_SPHel2DOF),-2(LElbow_SPHel2DOF),-3(LElbow_SPHel2DOF)>
	RElbow_SPHel2DOF = <1(RElbow_SPHel2DOF),2(RElbow_SPHel2DOF),3(RElbow_SPHel2DOF)>


{*
{*Miscellaneous Relative Angle Outputs*}

	LUpperArmtoPelAngle = <1(LUpperArmtoPelAngle),-2(LUpperArmtoPelAngle),-3(LUpperArmtoPelAngle)>
	RUpperArmtoPelAngle = <1(RUpperArmtoPelAngle),2(RUpperArmtoPelAngle),3(RUpperArmtoPelAngle)>
	PelShldrSepAng = <-1(PelShldrSepAng),2(PelShldrSepAng),3(PelShldrSepAng)>

{*Upper Arm outputs*}
{* Spherical ISB shoulder angles 								*}
{* Left shoulders have left handed spherical CS, where as right	*} 
{* shoulders have right handed spherical CS, i.e.: 				*}
{* Pole angle or plane of elevations (longitudes) are			*}
{*	Clkwise Movement to midline is +ve on left            	 	*}
{* 	AntiClkwise Movement to midline is +ve on right       		*}
{* Elevations (latitudes) are +ve up and zero is when y-axes of	*}
{* 	parent and upperarm are aligned. Same for left and 			*}
{*	right. This means that there are no -ve values for 			*}
{*	elevation.													*}
{* Long axis rotations depend on what quadrant the upperarm		*}
{* 	is in.														*} 
             
	pLShoulderAnatomyYXY = <-pLShoulderAnatomyYXY(1), pLShoulderAnatomyYXY(2), -pLShoulderAnatomyYXY(3)>
	pLShoulderThoraxYXY  = <-pLShoulderThoraxYXY(1), pLShoulderThoraxYXY(2), -pLShoulderThoraxYXY(3)>
	pLShoulderSWYXY      = <-pLShoulderSWYXY(1), pLShoulderSWYXY(2), -pLShoulderSWYXY(3)>
	pRShoulderAnatomyYXY = <pRShoulderAnatomyYXY(1), pRShoulderAnatomyYXY(2), pRShoulderAnatomyYXY(3)>
	pRShoulderThoraxYXY  = <pRShoulderThoraxYXY(1), pRShoulderThoraxYXY(2), pRShoulderThoraxYXY(3)>
	pRShoulderSWYXY      = <pRShoulderSWYXY(1), pRShoulderSWYXY(2), pRShoulderSWYXY(3)>

	{* Shoulder axes of rotation are defined in the following order, Flex/Extens, Abd/Add, Int/Ext Rotation *}
	peLShtoThoraxAngle = <-1(peLShtoThoraxAngle),-2(peLShtoThoraxAngle),3(peLShtoThoraxAngle)>
	peRShtoThoraxAngle = <1(peRShtoThoraxAngle),2(peRShtoThoraxAngle),3(peRShtoThoraxAngle)> 


	{*Relative Outputs for Tennis Racquet*}
	If $HandDom<>0
		RacqAngle = <1(RacqAngle),2(RacqAngle),3(RacqAngle)>
	Else
		RacqAngle = <1(RacqAngle),-2(RacqAngle),-3(RacqAngle)>
	EndIf

*}

{*Global Output Angles*}

	OUTPUT(PelvisAngle,PelvisAngleBaker,TrunkAngleGlob,TrunkAngleBaker,TorsoAngle,TorsoAngleBaker,ThoraxAngle,ThoraxAngleBaker)
	OUTPUT(RShWingAngleGlob,LShWingAngleGlob)
	OUTPUT(AnatRUpperArmAngleGlob,AnatLUpperArmAngleGlob)
	OUTPUT(HelRUpperArmAngleGlob,HelLUpperArmAngleGlob)
	OUTPUT(RForearmAngleGlob,LForearmAngleGlob)
	OUTPUT(RHandAngleGlob,LHandAngleGlob)

{*Relative Output Angles*}

	OUTPUT(TorsotoPelAngle,TorsotoThorAngle,ThortoPelAngle)
	OUTPUT(peLShoulderAngle,peRShoulderAngle,RShWingAngle,LShWingAngle)
	OUTPUT(LElbow_YAnat3DOF,RElbow_YAnat3DOF,LElbow_ZAnat3DOF,RElbow_ZAnat3DOF)
	OUTPUT(LElbow_FEAnat2DOF,RElbow_FEAnat2DOF,LElbow_SPAnat2DOF,RElbow_SPAnat2DOF)
	OUTPUT(LElbow_YHel3DOF,RElbow_YHel3DOF,LElbow_ZHel3DOF,RElbow_ZHel3DOF)
	OUTPUT(LElbow_FEHel2DOF,RElbow_FEHel2DOF,LElbow_SPHel2DOF,RElbow_SPHel2DOF)
	OUTPUT(LWristAngle, RWristAngle)
	OUTPUT(ShldrAlignAngle,PelShldrSepAng)
	OUTPUT(LUpperArmtoPelAngle, RUpperArmtoPelAngle)
	OUTPUT(peLShtoThoraxAngle, peRShtoThoraxAngle)

{*
UpperBodyModelVer = {10,8,2016}
OUTPUT(UpperBodyModelVer)
$UpperBodyModelVer = 10082016
PARAM($UpperBodyModelVer)
*}
{* End of processing *}


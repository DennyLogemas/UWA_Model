{*VICON BodyLanguage (tm)*}

{*===================================================================================================================================*}
{*====================================================UWA UPPER BODY STATIC MODEL====================================================*}
{*===================================================================================================================================*}

{*
	1. Program-Name Creation-Date
	UWAUBStatic.mod    30/June/2016
	2. Original-Author (Email-Address)
	Multiple Contributors
	3. Last-Updated-By (Email-Address)
	Dan Cottam (daniel.cottam@research.uwa.edu.au)
	4. Short-Description
	The generic UWA upper body static bodybuilder model
	5. Notes
	Please see documents on Sharepoint named "Changes to UWA Lower and Upper Body Models (June, 2016)" and Version 1.1 UWA Model Changes (August 2016) for information on model changes.
	6. Modification-History
	Version  Author     Date                 Change
	1.1       DC    10/August/2016   DrawSegment macro altered to match UB Dynamic and LB models
*}

{*==============================================================*}
{*Start of macro section*}
{*==============================================================*}

Macro DrawGlobal(ScaleFactor)
{* draws in the global coordinate system *}
Oglobal={0,0,0}
Xglobal=ScaleFactor*{1,0,0}
Yglobal=ScaleFactor*{0,1,0}
Zglobal=ScaleFactor*{0,0,1}
GlobalSystem=[Oglobal,Yglobal-Oglobal,Xglobal-Oglobal,yzx]
OUTPUT(Xglobal,Yglobal,Zglobal,Oglobal)
EndMacro

{*======================*}

Macro DrawSegment(segm)
{* draws each individual segment coordinate system *}
O#segm={0,0,0}*segm
X#segm=O#segm+80*1(segm) 
Y#segm=O#segm+80*2(segm) 
Z#segm=O#segm+80*3(segm) 
Output(O#segm,X#segm,Y#segm,Z#segm)
EndMacro

{*======================*}

{*Pointer Macro*}

{*Assumes 6-marker pointer is being used*}

macro Pointer($PointingAt,paramname,P1,P2,P3,P4,P5,P6,segm)
origin=(P1+P2+P3+P4)/4
OUTPUT(origin)
directA=[origin,P1-P3,origin-P5,xyz]
directB=[origin,P2-P4,origin-P5,xyz]
directC=[origin,origin-P5,origin-P3,zyx]

{* The 6-marker Pointer is 355mm from origin to endpoint *}
Pntr1= origin+355*3(directA)
Pntr2= origin+355*3(directB)
Pntr3= origin+355*3(directC)
$PointingAt=(Pntr1+Pntr2+Pntr3)/3 ? Pntr1 ? Pntr2 ? Pntr3 

OUTPUT($PointingAt)
paramname=$PointingAt/segm
PARAM(paramname)
OUTPUT(paramname)
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
{* Defines UWA optional marker points. Some of these points may be used in the static *}
{* trial, but not the dynamic.  They include virtual points, ie. helical axes,*}
{* 'AnatRelTech' points and 'pointerRelTech' points *}

{* Head & Thorax Markers*}

	OptionalPoints(LFHD, RFHD, LBHD, RBHD)
	OptionalPoints(C7, T10, CLAV, STRN)
	OptionalPoints(VTR1, VTR2, VTR3, VTR4)

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

{* Pointer Markers*}
	
	OptionalPoints(PNTR1, PNTR2, PNTR3, PNTR4, PNTR5)
	OptionalPoints(PLLEL, PLMEL, PRLEL, PRMEL)
	OptionalPoints(PT10,PSTRN,PC7)
	
{* Anatomical Elbow Markers *}
	OptionalPoints(LLEL,LMEL,RLEL,RMEL)
	
{*Virtual Markers*}	
	OptionalPoints($LOptimalHip,$ROptimalHip)
	OptionalPoints($LMELpointerRelTech,$LLELpointerRelTech,$RMELpointerRelTech,$RLELpointerRelTech)

{* SCoRE/SARA Markers*}
	
	OptionalPoints(dLUA_pLFA_sara, dLUA_pLFA_score, dRUA_pRFA_sara, dRUA_pRFA_score)	
	OptionalPoints(pLFA_LFA_sara, pLFA_LFA_score, pRFA_RFA_sara, pRFA_RFA_score)
		
DrawGlobal(200)

{* END Initialisations*}

{* =================================================================================*}
{* 							Start Static Segmental Modelling		       		    *}
{* =================================================================================*}

{* =================================================================================*}
{* 									Head 		  	 				      		    *}
{* =================================================================================*}

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

	OUTPUT(HeadOrigin,HeadMid)
	DrawSegment(Head)

	HeightZ = $Height*{0,0,1}
	HeadMidXValue = HeadMid(1)
	HeadMidXPoint = HeadMidXValue*{1,0,0}
	HeadMidYValue = HeadMid(2)
	HeadMidYPoint = HeadMidYValue*{0,1,0}
	TopHead= HeightZ+HeadMidXPoint+HeadMidYPoint
	PARAM(TopHead,HeadMid)
	OUTPUT(TopHead)

	$TopHeadRelTech=TopHead/Head
	PARAM($TopHeadRelTech)


{*================================================================*}
{*		       				    Pelvis	                          *}
{*================================================================*}

{* Pelvis and HJC's defined for femur replace macro in femur if required *}
{* Uses HJC location of Orthotrak (Shea et al.1997 Gait and Posture 5,157) *}
{* Even though this is essentially dynamic code, it is defined here so HJC can be used for femur REPLACE Macro to follow*}
	
	If EXISTATALL(LPSI)
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
				 -(0.34*InterASISDist),
				 (-0.32*InterASISDist)}*Pelvis
		RHJC = {-(0.21*InterASISDist)-mm,
				 -(0.34*InterASISDist),
				 (0.32*InterASISDist)}*Pelvis
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

{*HipJoints (not drawn)*}

	LHipJoint = LHJC+Attitude(Pelvis)
	RHipJoint = RHJC+Attitude(Pelvis)

{* =============================================================================================*}
{* 					Trunk, Thorax & Torso				    		*}
{* =============================================================================================*}
{* use this replace macro if there are missing thorax or torso markers, which cannot be interpolated *}
{* USE WISELY!!*}
{*SUBSTITUTE4(STRN,CLAV,C7,T10)*}
{*SUBSTITUTE4(STRN,C7,RACR,T10)*}
{*SUBSTITUTE4(CLAV,C7,RACR,T10)*}
{*SUBSTITUTE4(RACR,CLAV,C7,T10)*}
{*SUBSTITUTE4(RACR,CLAV,C7,STRN)*}

{*===================================Virtual Thorax===================================*}

{*Creates dummy thorax*}
	
	VirtualThoraxOrigin=(VTR1+VTR2+VTR3)/3
	DummyThorax=[VirtualThoraxOrigin,VTR2-VirtualThoraxOrigin,VTR1-VTR3,yxz]
	OUTPUT(VirtualThoraxOrigin)
	
	If ExistAtAll (VTR4)
		VirtualThoraxOrigin2=(VTR2+VTR3+VTR4)/3
		DummyThorax2=[VirtualThoraxOrigin2,VTR2-VTR4,VirtualThoraxOrigin2-VTR3,yxz]
		$VTR1RelTech=VTR1/DummyThorax2
		PARAM($VTR1RelTech)

		VirtualThoraxOrigin3=(VTR1+VTR3+VTR4)/3
		DummyThorax3=[VirtualThoraxOrigin3,VirtualThoraxOrigin3-VTR4,VTR1-VTR3,yxz]
		$VTR2RelTech=VTR2/DummyThorax3
		PARAM($VTR2RelTech)
		
		VirtualThoraxOrigin4=(VTR1+VTR2+VTR4)/3
		DummyThorax4=[VirtualThoraxOrigin4,VTR2-VTR4,VTR1-VirtualThoraxOrigin4,yxz]
		$VTR3RelTech=VTR3/DummyThorax4
		PARAM($VTR3RelTech)
	EndIf
	
{*Pointer T10*}
	
	If ExistAtAll(PT10,VirtualThoraxOrigin)
		Pointer(VT10,$VT10pointerRelTech,PNTR1,PNTR2,PNTR3,PNTR4,PNTR5,PT10,DummyThorax)
	EndIf

{*Pointer Sternum*}
	
	If ExistAtAll(PSTRN,VirtualThoraxOrigin)
		Pointer(VSTRN,$VSTRNpointerRelTech,PNTR1,PNTR2,PNTR3,PNTR4,PNTR5,PSTRN,DummyThorax)
	EndIf
	
{*Pointer C7*}	
	
	If ExistAtAll(PC7,VirtualThoraxOrigin)
		Pointer(VC7,$VC7pointerRelTech,PNTR1,PNTR2,PNTR3,PNTR4,PNTR5,PC7,DummyThorax)
	EndIf
	
{* Reconstructs pointer defined marker positions by reading the PARAM values created after running the pointer macro in the pointer section above.*}

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
	
{*Create Thorax Segment. Allowance for virtual C7, T10 and sternum markers added by Dan Cottam, June 2016*}

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

{*Create Trunk Segment. A previously used definition of the trunk is saved in the UWA legacy code mod file. Trunk segment added by Dan Cottam, June 2016*}

	If ExistAtAll (VC7)
		Trunk = [MidHJC,RHJC-LHJC,VC7-MidHJC,zxy]
	ELSE Trunk = [MidHJC,RHJC-LHJC,C7-MidHJC,zxy]
	EndIf
	
	DrawSegment (Trunk)	
	
{*Calculates Trunk Length. Allowance for VT10, VSTRN & VC7 added June 2016*}

{*
Average functions are not permitted in If statements. 
Therefore the different permutations of TrunkLength need to be calculated separately before being renamed as "TrunkLength" in an If statement
*}

	C7TrunkLength=Average(DIST(C7,MidHJC))
	VC7TrunkLength=Average(DIST(VC7,MidHJC))

	If ExistAtAll (VC7)
		$TrunkLength=VC7TrunkLength
	ELSE $TrunkLength=C7TrunkLength
	PARAM($TrunkLength)
	EndIf	
	
{* Calculate thorax length *}

{*
Average functions are not permitted in If statements. 
Therefore the different permutations of ThoraxLength need to be calculated separately before being renamed as "ThoraxLength" in an If statement
*}

	ThoraxLength1= Average(DIST(VC7,VSTRN))
	ThoraxLength2= Average(DIST(C7,VSTRN))
	ThoraxLength3= Average(DIST(VC7,STRN))
	ThoraxLength4= Average(DIST(C7,STRN))
	
	If ExistAtAll (VC7,VSTRN)
		ThoraxLength = ThoraxLength1
	ELSIF ExistAtAll (C7,VSTRN)
		ThoraxLength = ThoraxLength2
	ELSIF ExistAtAll (VC7,STRN)
		ThoraxLength = ThoraxLength3
	ELSE ThoraxLength = ThoraxLength4
	EndIf
	$ThoraxLength=ThoraxLength
	PARAM($ThoraxLength)
	
{* Create Virtual C7, MidHip Joint Centre and Sternum points relative to respective Segments *}
{*
	CreateRelPoint(C7,Thorax,C7relThorax)
	OUTPUT(C7relThorax)

	MidHJC=(LHJC+RHJC)/2
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

	MidShldr=(RACR+LACR)/2
	
	RAcromionOrigin = RACR
	LAcromionOrigin = LACR
	RAcromion = [RAcromionOrigin,RACR3-RACR1,RACR-RACR2,xzy]
	LAcromion = [LAcromionOrigin,LACR3-LACR1,LACR-LACR2,xzy]
	DrawSegment(RAcromion)
	DrawSegment(LAcromion)

{* =====================================================================================*}
{* 					Proximal Upper Arm			    									*}
{* =====================================================================================*}

{*Fourth marker added June 2016. Stores position of markers 1-3 in mp by creating coordinate systems using the fourth marker*}

{*Right Proximal Upper Arm*}

	pRightUpperArmOrigin=(RUA1+RUA2+RUA3)/3
	pRightUpperArm = [pRightUpperArmOrigin,RUA1-RUA3,RUA2-pRightUpperArmOrigin, yzx] 
	OUTPUT(pRightUpperArmOrigin)
	
	DrawSegment(pRightUpperArm)

	If ExistAtAll (RUA4)
		pRightUpperArmOrigin2=(RUA2+RUA3+RUA4)/3
		pRightUpperArm2 = [pRightUpperArmOrigin2,pRightUpperArmOrigin2-RUA3,RUA2-RUA4,yzx]
		$RUA1RelTech=RUA1/pRightUpperArm2
		PARAM($RUA1RelTech)
		
		pRightUpperArmOrigin3=(RUA1+RUA3+RUA4)/3
		pRightUpperArm3 = [pRightUpperArmOrigin3,RUA1-RUA3,RUA4-pRightUpperArmOrigin3,yzx]
		$RUA2RelTech=RUA2/pRightUpperArm3
		PARAM($RUA2RelTech)
		
		pRightUpperArmOrigin4=(RUA1+RUA2+RUA4)/3
		pRightUpperArm4 = [pRightUpperArmOrigin4,RUA1-pRightUpperArmOrigin4,RUA2-RUA4,yzx]
		$RUA3RelTech=RUA3/pRightUpperArm4
		PARAM($RUA3RelTech)
	EndIf
	
{*Left Proximal Upper Arm*}

	pLeftUpperArmOrigin=(LUA1+LUA2+LUA3)/3
	pLeftUpperArm = [pLeftUpperArmOrigin,LUA1-LUA3,LUA2-pLeftUpperArmOrigin,yzx] 
	OUTPUT(pLeftUpperArmOrigin)
	
	DrawSegment(pLeftUpperArm)

	If ExistAtAll (LUA4)
		pLeftUpperArmOrigin2=(LUA2+LUA3+LUA4)/3
		pLeftUpperArm2 = [pLeftUpperArmOrigin2,pLeftUpperArmOrigin2-LUA3,LUA2-LUA4,yzx]
		$LUA1RelTech=LUA1/pLeftUpperArm2
		PARAM($LUA1RelTech)
		
		pLeftUpperArmOrigin3=(LUA1+LUA3+LUA4)/3
		pLeftUpperArm3 = [pLeftUpperArmOrigin3,LUA1-LUA3,LUA4-pLeftUpperArmOrigin3,yzx]
		$LUA2RelTech=LUA2/pLeftUpperArm3
		PARAM($LUA2RelTech)
		
		pLeftUpperArmOrigin4=(LUA1+LUA2+LUA4)/3
		pLeftUpperArm4 = [pLeftUpperArmOrigin4,LUA1-pLeftUpperArmOrigin4,LUA2-LUA4,yzx]
		$LUA3RelTech=LUA3/pLeftUpperArm4
		PARAM($LUA3RelTech)
	EndIf

{* =================================================================================*}
{* 		Shoulder Joint, Wings and Single Segment			           				*}
{* =================================================================================*}

{* 						SJC - Static Marker Method 									*}

{* 
Original method of calculating the SJC: Accuracy relative to MRI SJC location = 14 mm - 
(this was with a very practiced tester i.e. worse errors were recorded!) Very dependent on tester.
inter-tester reliability error 17 mm,inter-tester reliability error 9 mm. Has been commented out. Only use if last resort.
*}  
{*
IF EXISTATALL(LPSH,LASH)

	LSJC = PERP(LACR,LPSH,LASH) 
	LSJC = (LACR+LPSH+LASH)/3
	$LSJCA = LSJC/LAcromion
	$LSJCB = LSJC/pLeftUpperArm
	PARAM($LSJCA)
	PARAM($LSJCB)
	OUTPUT(LSJC)
		ELSE
	LSJC = $LSJCAnatRelTech*LAcromion
	OUTPUT(LSJC)
	
ENDIF
	
IF EXISTATALL(RPSH,RASH)

	RSJC = PERP(RACR,RPSH,RASH)
	RSJC = (RACR+RPSH+RASH)/3
	$RSJCA = RSJC/RAcromion
	$RSJCB = RSJC/pRightUpperArm
	PARAM($RSJCA)
	PARAM($RSJCB)
	OUTPUT(RSJC)
		ELSE
	RSJC = $RSJCAnatRelTech*RAcromion
	OUTPUT(RSJC)
	
ENDIF
*}

{* 						SJC - Generic Offset Method 								*}

{* 3D Generic offset method method of calculating the SJC is simply the average location of the SJC in 15 healthy males in MRI. 
Average height and mass of this sample was 177.6cm (range: 23.7cm) and 75.9kg (range: 48.2kg). 
Don't use this method on kids/elderly.
Accuracy relative to MRI was 12 mm. Inter-tester reliability error 13 mm,inter-tester reliability error 9 mm *} 

If $ShoulderModel==1

	LACR/LAcromion
	LSJC = {(LACR(1)+12),(LACR(2)-6),(LACR(3)-49)}
	$LSJCA = LSJC/LAcromion
	$LSJCB = LSJC/pLeftUpperArm
	PARAM($LSJCA)
	PARAM($LSJCB)
	OUTPUT(LSJC)
	
	RACR/RAcromion
	RSJC = {(RACR(1)+12),(RACR(2)+6),(RACR(3)-49)}
	$RSJCA = RSJC/RAcromion
	$RSJCB = RSJC/pRightUpperArm
	PARAM($RSJCA)
	PARAM($RSJCB)
	OUTPUT(RSJC)
	
ENDIF


{* 						SJC - Regression Equation Method 							*}

{* This method of calculating the SJC is based on subject height, mass, the C7,STRN and Acromion triad. 
Average height and mass of the sample used to develop these multiple regression equations was the same as above. i.e.
Don't use this method on kids/elderly.
Accuracy 12 mm. Inter-tester reliability error 6 mm,inter-tester reliability error 6 mm 
X =  96.2 - 0.302 * (CLAV-C7) - 0.364 * height + 0.385 * mass  
y = -66.32 + 0.309*(CLAV-C7) - 0.432 * mass 	
z = 66.468 - 0.531 *(AcrLR-CP) + 0.571* mass	
In the code below, dist1 = Clav-C7, dist2 = AcrLR-CP *}


If $ShoulderModel==2

      ht = $Height/10
      mass = $BodyMass
      D1 = DIST(CLAV,C7)
      CP = (CLAV+C7)/2
      D2L = DIST(LACR,CP)
      D2R = DIST(RACR,CP)

      LSJC = {96.2-(0.302*D1)-(0.364*ht)+(0.385*mass),
            -66.32+(0.309*D1)-(0.432*mass),
            -(66.468-(0.531*D2L)+(0.571*mass))}*LAcromion

    $LSJCA = LSJC/LAcromion
	$LSJCB = LSJC/pLeftUpperArm 
	PARAM($LSJCA)
	PARAM($LSJCB)
	OUTPUT(LSJC)
      
      RSJC = {96.2-(0.302*D1)-(0.364*ht)+(0.385*mass),
            -66.32+(0.309*D1)-(0.432*mass),
            66.468-(0.531*D2R)+(0.571*mass)}*RAcromion
      
    $RSJCA = RSJC/RAcromion
	$RSJCB = RSJC/pRightUpperArm 
	PARAM($RSJCA)
	PARAM($RSJCB)
	OUTPUT(RSJC)

  
ENDIF

{* Create a Shoulder "Wing" Segment, which is a plane created from midpoint between C7 and CLAV (ThoraxOrigin), 
   Shoulder marker (xACR) and modelled shoulder joint centre (xSJC) *}

	LeftShoulderWing = [LSJC, ThoraxOrigin-LSJC, LACR-LSJC, zxy]
	RightShoulderWing = [RSJC, RSJC-ThoraxOrigin, RACR-RSJC, zxy]

{* 
Create a single shoulder segment to allow calculation of hip-shoulder separation (HSS) angle 
 and comparison to previous trunk research (e.g. fast bowling back injury work
*} 

	If exist(RSJC ? LSJC)
		MidShldr = (LSJC+RSJC)/2
		ShldrAlign = [MidShldr,RSJC-LSJC,C7-ThoraxOrigin,zyx]
	else
		MidShldr = (RACR+LACR)/2
		ShldrAlign = [MidShldr,RACR-LACR,C7-ThoraxOrigin,zyx]	
	Endif	

	OUTPUT(MidShldr)
	DrawSegment(LeftShoulderWing)
	DrawSegment(RightShoulderWing)
	DrawSegment(ShldrAlign)

{*======================================================================================*}
{*   						Distal Upper Arm Segment  											*}
{*======================================================================================*}
{*Fourth marker added June 2016*}

{*Right Distal Upper Arm*}

	dRightUpperArmOrigin=(dRUA1+dRUA2+dRUA3)/3
	dRightUpperArm = [dRightUpperArmOrigin,dRUA1-dRUA3,dRUA2-dRightUpperArmOrigin,zxy] 
	OUTPUT(dRightUpperArmOrigin)
	
	DrawSegment(dRightUpperArm)

	If ExistAtAll (dRUA4)
		dRightUpperArmOrigin2=(dRUA2+dRUA3+dRUA4)/3
		dRightUpperArm2 = [dRightUpperArmOrigin2,dRightUpperArmOrigin2-dRUA3,dRUA2-dRUA4,zxy]
		$dRUA1RelTech=dRUA1/dRightUpperArm2
		PARAM($dRUA1RelTech)
		
		dRightUpperArmOrigin3=(dRUA1+dRUA3+dRUA4)/3
		dRightUpperArm3 = [dRightUpperArmOrigin3,dRUA1-dRUA3,dRUA4-dRightUpperArmOrigin3,zxy]
		$dRUA2RelTech=dRUA2/dRightUpperArm3
		PARAM($dRUA2RelTech)
		
		dRightUpperArmOrigin4=(dRUA1+dRUA2+dRUA4)/3
		dRightUpperArm4 = [dRightUpperArmOrigin4,dRUA1-dRightUpperArmOrigin4,dRUA2-dRUA4,zxy]
		$dRUA3RelTech=dRUA3/dRightUpperArm4
		PARAM($dRUA3RelTech)
	EndIf

{*Left Distal Upper Arm*}
	
	dLeftUpperArmOrigin=(dLUA1+dLUA2+dLUA3)/3
	dLeftUpperArm = [dLeftUpperArmOrigin,dLUA1-dLUA3,dLUA2-dLeftUpperArmOrigin,zxy] 
	OUTPUT(dLeftUpperArmOrigin)
	
	DrawSegment(dLeftUpperArm)

	If ExistAtAll (dLUA4)
		dLeftUpperArmOrigin2=(dLUA2+dLUA3+dLUA4)/3
		dLeftUpperArm2 = [dLeftUpperArmOrigin2,dLeftUpperArmOrigin2-dLUA3,dLUA2-dLUA4,zxy]
		$dLUA1RelTech=dLUA1/dLeftUpperArm2
		PARAM($dLUA1RelTech)
		
		dLeftUpperArmOrigin3=(dLUA1+dLUA3+dLUA4)/3
		dLeftUpperArm3 = [dLeftUpperArmOrigin3,dLUA1-dLUA3,dLUA4-dLeftUpperArmOrigin3,zxy]
		$dLUA2RelTech=dLUA2/dLeftUpperArm3
		PARAM($dLUA2RelTech)
		
		dLeftUpperArmOrigin4=(dLUA1+dLUA2+dLUA4)/3
		dLeftUpperArm4 = [dLeftUpperArmOrigin4,dLUA1-dLeftUpperArmOrigin4,dLUA2-dLUA4,zxy]
		$dLUA3RelTech=dLUA3/dLeftUpperArm4
		PARAM($dLUA3RelTech)
	EndIf

{*======================================================================================*}
{* 				Elbow Anat Landmark Section	(formerly Pointer Section)					*}
{*======================================================================================*}

If $Pointer==1

	{*Pointer Storage - Run pointer macro for elbow epicondyles*}
	
		If ExistAtAll(PLLEL)	
			Pointer(LeLEL,$LLELpointerRelTech,PNTR1,PNTR2,PNTR3,PNTR4,PNTR5,PLLEL,dLeftUpperArm)
		EndIf

		If ExistAtAll(PLMEL)
			Pointer(LeMEL,$LMELpointerRelTech,PNTR1,PNTR2,PNTR3,PNTR4,PNTR5,PLMEL,dLeftUpperArm)
		EndIf

		If ExistAtAll(PRLEL)
			Pointer(RiLEL,$RLELpointerRelTech,PNTR1,PNTR2,PNTR3,PNTR4,PNTR5,PRLEL,dRightUpperArm)
		EndIf

		If ExistAtAll(PRMEL)
			Pointer(RiMEL,$RMELpointerRelTech,PNTR1,PNTR2,PNTR3,PNTR4,PNTR5,PRMEL,dRightUpperArm)
		EndIf

	{*Pointer Output - Pointer defined epicondyle markers*}

		RiMEL= $RMELpointerRelTech*dRightUpperArm
		OUTPUT(RiMEL)
		
		RiLEL= $RLELpointerRelTech*dRightUpperArm
		OUTPUT(RiLEL)

		LeMEL=$LMELpointerRelTech*dLeftUpperArm
		OUTPUT(LeMEL)			

		LeLEL=$LLELpointerRelTech*dLeftUpperArm
		OUTPUT(LeLEL)
EndIf

If $Pointer==0	

	{*Marker Storage - Store anat epicondyle mkrs*}
		
		If ExistAtAll(LLEL)	
			$LeLEL = LLEL/dLeftUpperArm
		EndIf
		PARAM($LeLEL)

		If ExistAtAll(LMEL)
			$LeMEL = LMEL/dLeftUpperArm
		EndIf
		PARAM($LeMEL)

		If ExistAtAll(RLEL)
			$RiLEL = RLEL/dRightUpperArm
		EndIf
		PARAM($RiLEL)

		If ExistAtAll(RMEL)
			$RiMEL = RMEL/dRightUpperArm
		EndIf
		PARAM($RiMEL)
		
	{*Marker Output - Outputs anat epicondyle markers*}

		RiMEL= $RiMEL*dRightUpperArm
		OUTPUT(RiMEL)
		
		RiLEL= $RiLEL*dRightUpperArm
		OUTPUT(RiLEL)

		LeMEL=$LeMEL*dLeftUpperArm
		OUTPUT(LeMEL)			

		LeLEL=$LeLEL*dLeftUpperArm
		OUTPUT(LeLEL)
		
EndIf
		
{*======================================================================================*}
{*     Define Elbow Joint Centres     *}
{*======================================================================================*}

{* Use parameter HelicalElbow to define the approach *}
{* HelicalElbow = 0: use virtual markers created from pointer trials *}
{* HelicalElbow = 1: SCoRE/SARA elbow FE *}
{* HelicalElbow = 2: SCoRE/SARA elbow FE + PS **NOT COMPLETE** *}


	LMidEp=($LeLEL+$LeMEL)/2
	RMidEp=($RiLEL+$RiMEL)/2
	
	LMidEp=LMidEp*dLeftUpperArm
	RMidEp=RMidEp*dRightUpperArm

	PARAM(LMidEp,RMidEp)
	
	If $HelicalElbow==0
	
		LEJC=LMidEp
		REJC=RMidEp
	
		OUTPUT(LEJC,REJC)
	
	EndIf
	
	
	If $HelicalElbow==1
	{* Stores SCoRE/SARA calculated EJC + FE axis of rotation for use with dynamic UWA code, added by D Wells April 2018 *}

		If ExistAtAll(dRUA_pRFA_score)
		
			$REHA1 = dRUA_pRFA_score/dRightUpperArm
			$REHA2 = dRUA_pRFA_sara/dRightUpperArm
			PARAM($REHA1,$REHA2)

		EndIf	

		If ExistAtAll(dLUA_pLFA_score)
		
			$LEHA1 = dLUA_pLFA_score/dLeftUpperArm
			$LEHA2 = dLUA_pLFA_sara/dLeftUpperArm
			PARAM($LEHA1,$LEHA2)

		EndIf	
		
		LEHel1=$LEHA1*dLeftUpperArm
		LEHel2=$LEHA2*dLeftUpperArm  
		REHel1=$REHA1*dRightUpperArm
		REHel2=$REHA2*dRightUpperArm
		OUTPUT(LEHel1,LEHel2,REHel1,REHel2)
		
		LEJC=PERP(LMidEp,LEHel1,LEHel2)
		REJC=PERP(RMidEp,REHel1,REHel2)
		
		OUTPUT(LEJC,REJC)
		
	EndIf
	
{*==============================================================================*}
{* 								Forearm    										*}
{*==============================================================================*}
{*Fourth marker added June 2016*}

{*create dummy right forearm segment*}

	RightForearmOrigin=(RFA1+RFA2+RFA3)/3
	DummyRightForearm = [RightForearmOrigin,RFA1-RFA3,RFA2-RightForearmOrigin,zxy]  
	OUTPUT(RightForearmOrigin)

	If ExistAtAll (RFA4)
		RightForearmOrigin2=(RFA2+RFA3+RFA4)/3
		DummyRightForearm2 = [RightForearmOrigin2,RightForearmOrigin2-RFA3,RFA2-RFA4,zxy]
		$RFA1RelTech=RFA1/DummyRightForearm2
		PARAM($RFA1RelTech)
		
		RightForearmOrigin3=(RFA1+RFA3+RFA4)/3
		DummyRightForearm3 = [RightForearmOrigin3,RFA1-RFA3,RFA4-RightForearmOrigin3,zxy]
		$RFA2RelTech=RFA2/DummyRightForearm3
		PARAM($RFA2RelTech)
		
		RightForearmOrigin4=(RFA1+RFA2+RFA4)/3
		DummyRightForearm4 = [RightForearmOrigin4,RFA1-RightForearmOrigin4,RFA2-RFA4,zxy]
		$RFA3RelTech=RFA3/DummyRightForearm4
		PARAM($RFA3RelTech)
	EndIf

{*create dummy left forearm segment*}	
	
	LeftForearmOrigin=(LFA1+LFA2+LFA3)/3
	DummyLeftForearm = [LeftForearmOrigin,LFA1-LFA3,LFA2-LeftForearmOrigin,zxy]  
	OUTPUT(LeftForearmOrigin)

	If ExistAtAll (LFA4)
		LeftForearmOrigin2=(LFA2+LFA3+LFA4)/3
		DummyLeftForearm2 = [LeftForearmOrigin2,LeftForearmOrigin2-LFA3,LFA2-LFA4,zxy]
		$LFA1RelTech=LFA1/DummyLeftForearm2
		PARAM($RFA1RelTech)
		
		LeftForearmOrigin3=(LFA1+LFA3+LFA4)/3
		DummyLeftForearm3 = [LeftForearmOrigin3,LFA1-LFA3,LFA4-LeftForearmOrigin3,zxy]
		$LFA2RelTech=LFA2/DummyLeftForearm3
		PARAM($LFA2RelTech)
		
		LeftForearmOrigin4=(LFA1+LFA2+LFA4)/3
		DummyLeftForearm4 = [LeftForearmOrigin4,LFA1-LeftForearmOrigin4,LFA2-LFA4,zxy]
		$LFA3RelTech=LFA3/DummyLeftForearm4
		PARAM($LFA3RelTech)
	EndIf
	 
{* Store wrist axis positions relative to dummy forearm segments. These can be recalled in the dynamic model if an actual wrist marker is missing. *}

	$LWRRAnatRelTech=LWRR/DummyLeftForearm
	$LWRUAnatRelTech=LWRU/DummyLeftForearm
	PARAM($LWRRAnatRelTech,$LWRUAnatRelTech)

	$RWRRAnatRelTech=RWRR/DummyRightForearm
	$RWRUAnatRelTech=RWRU/DummyRightForearm
	PARAM($RWRRAnatRelTech,$RWRUAnatRelTech)

{* Find Wrist joint centres using real anatomical markers*}

	LWJC = (LWRR+LWRU)/2
	RWJC = (RWRR+RWRU)/2	
	
	OUTPUT(LWJC,RWJC)

{*======================================================================================*}
{* 					Hand               				*}
{*======================================================================================*}

{* Each hand has 3 real markers (LHNR,LHNU,LCAR or RHNR, RHNU,RCAR)*}
{* and 2 static joint markers (LWRR, LWRU, or RWRR, WWRU)     *}
{* and 1 virtual marker (LWJC or RWJC)                        *}

{* only use this replace macro if desperate and you have missing HAND triad markers!*}

If EXISTATALL(LCAR,RCAR)

	LeftHandOrigin =(LHNR+LHNU+LCAR)/3
	RightHandOrigin = (RHNR+RHNU+RCAR)/3
		ELSE
	LeftHandOrigin =(LHNR+LHNU)/2
	RightHandOrigin = (RHNR+RHNU)/2
	
ENDIF

LeftHand= [LeftHandOrigin,LWJC-LeftHandOrigin,LHNR-LHNU,yxz]
RightHand= [RightHandOrigin, RWJC-RightHandOrigin, RHNU-RHNR,yxz]

OUTPUT(LeftHandOrigin,RightHandOrigin)
DrawSegment(LeftHand)
DrawSegment(RightHand)

{* Hand lengths are required for Kinetics, where hand COM is defined from WJC to 3rd Metacarpal*}

If $LHandLength==0 

	$LHandLength =DIST(LCAR,LWJC) 
	Else
	$LHandLength = $LHandLength
	
EndIf

If $RHandLength==0 

	$RHandLength = DIST(RCAR,RWJC) 
	Else
	$RHandLength = $RHandLength
	
EndIf

PARAM($LHandLength,$RHandLength)

{*
UpperBodyModelVer = {10,9,2016}
OUTPUT(UpperBodyModelVer)
$UpperBodyModelVer = 10092016
PARAM($UpperBodyModelVer)
*}
{* End of processing *}
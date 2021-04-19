{*VICON BodyLanguage (tm)*}

{*===================================================================================================================================*}
{*====================================================UWA LOWER BODY STATIC MODEL====================================================*}
{*===================================================================================================================================*}

{*
	1. Program-Name Creation-Date
	UWALBStatic.mod  30/June/2016
	2. Original-Author (Email-Address)
	Multiple Contributors
	3. Last-Updated-By (Email-Address)
	Dan Cottam (daniel.cottam@research.uwa.edu.au)
	4. Short-Description
	The generic UWA lower body static bodybuilder model
	5. Notes
	Please see documents on Sharepoint named "Changes to UWA Lower and Upper Body Models (June, 2016)" and Version 1.1 UWA Model Changes (August 2016) for information on model changes.
	To use SCoRE/SARA functional hip and knee definitions enter '2' into Subject Parameters 'OptimalHip' and 'HelicalKnee' (D Wells May 2017)
	6. Modification-History
	Version  Author     Date                 Change
	1.1       DC    10/August/2016   Baker angle outputs corrected
*}

{*==============================================================*}
{*Start of macro section*}
{*==============================================================*}

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
Output(O#segm,X#segm,Y#segm,Z#segm)
EndMacro

{* ============ MACRO CREATE RELATIVE POINT ============*}
Macro CreateRelPoint(PGlob,segm,NewPointName)
{*Creates a new point relative to a local segement*}
PLocal=PGlob/segm
PLocalYValue=PLocal(2)
PLocalYPoint=PLocalYValue*{0,1,0}
NewPointName=PLocalYPoint*segm
EndMacro

{*======================*}
{*Pointer Macro*}

{*Assumes 6-marker pointer is being used. 5-marker pointer macro is kept in legacy model file*}

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

{*End of macro section*}
{*==============================================================*}

{*==============================================================*}
{* START Initialisations*}
{*==============================================================*}
{* 
Defines UWA.mod optional marker points. These points will be used in the static
trial, but not the dynamic. Only points which are not otherwise defined in the script need to be listed here. For example, virtual elbow epicondyles do not need to be included here because they are defined in the code.
*}

{* Head, Shoulder and Thorax Markers*}
	
	OptionalPoints(LFHD, RFHD, LBHD, RBHD)
	OptionalPoints(C7, T10, CLAV, STRN)
	OptionalPoints(VTR1, VTR2, VTR3, VTR4)
	OptionalPoints(RACR, RASH, RPSH)
	OptionalPoints(LACR, LASH, LPSH) 
	
{* Pelvis Markers*}
	
	OptionalPoints(LPSI, RPSI, LASI, RASI)

{* Left Leg Markers*}
	
	OptionalPoints(LTH1, LTH2, LTH3, LTH4)
	OptionalPoints(LLFC, LMFC)
	OptionalPoints(LTB1, LTB2, LTB3, LTB4)
	OptionalPoints(LLMAL, LMMAL)
	OptionalPoints(LMT1, LMT5, LCAL)
	OptionalPoints(LHEE)

{* Right Leg Markers*}
	
	OptionalPoints(RTH1, RTH2, RTH3, RTH4)
	OptionalPoints(RLFC, RMFC)
	OptionalPoints(RTB1, RTB2, RTB3, RTB4)
	OptionalPoints(RLMAL, RMMAL)
	OptionalPoints(RMT1, RMT5, RCAL)
	OptionalPoints(RHEE)

{* Helical Axis Markers*}
	
	OptionalPoints($LKHA1,$LKHA2,$RKHA1,$RKHA2)
	OptionalPoints($ROptimalHip,$LOptimalHip)

{* Foot Rig Markers*}
	
	OptionalPoints(RIGR, RIGL, RIGA, RIGP)

{* Pointer Markers*}
	
	OptionalPoints(PNTR1, PNTR2, PNTR3, PNTR4,PNTR5)
	OptionalPoints(PLLFC, PLMFC, PRLFC, PRMFC, PT10, PSTRN, PC7)
	
{* SCoRE/SARA Markers*}
	
	OptionalPoints(Pelvis_LThigh_score, Pelvis_RThigh_score)	
	OptionalPoints(LThigh_LTibia_score, LThigh_LTibia_sara, RThigh_RTibia_score, RThigh_RTibia_sara)
	
DrawGlobal(200)

{* END Initialisations*}
{*==============================================================*}

{*================================================================*}
{*==============Lower Body Static Pointer Section=================*}
{*================================================================*}

{*This section creates dummy left and right femurs. Fourth marker added by Dan Cottam, June 2016 *}

{* 
Creates the dummy segment using markers 1-3. Using the fourth marker, the position of the first three markers is also stored in the mp file. 
Any of the first three markers are recreated in the dynamic model if one is missing from the trial.
*} 

{*Dummy Left Femur*}
	
	LeftThighOrigin=(LTH1+LTH2+LTH3)/3
	DummyLeftFemur = [LeftThighOrigin,LTH1-LTH3,LTH2-LeftThighOrigin,yzx]
	OUTPUT(LeftThighOrigin)

	If ExistAtAll(LTH4)
		LeftThighOrigin2=(LTH2+LTH3+LTH4)/3
		DummyLeftFemur2 = [LeftThighOrigin2,LeftThighOrigin2-LTH3,LTH2-LTH4,yzx]
		$LTH1RelTech=LTH1/DummyLeftFemur2
		PARAM($LTH1RelTech)
		
		LeftThighOrigin3=(LTH1+LTH3+LTH4)/3
		DummyLeftFemur3 = [LeftThighOrigin3,LTH1-LTH3,LTH4-LeftThighOrigin3,yzx]
		$LTH2RelTech=LTH2/DummyLeftFemur3
		PARAM($LTH2RelTech)
		
		LeftThighOrigin4=(LTH1+LTH2+LTH4)/3
		DummyLeftFemur4 = [LeftThighOrigin4,LTH1-LeftThighOrigin4,LTH2-LTH4,yzx]
		$LTH3RelTech=LTH3/DummyLeftFemur4
		PARAM($LTH3RelTech)
	EndIf
	
	
{*Dummy Right Femur*}
	
	RightThighOrigin=(RTH1+RTH2+RTH3)/3
	DummyRightFemur = [RightThighOrigin,RTH1-RTH3,RTH2-RightThighOrigin,yzx]
	OUTPUT(RightThighOrigin)

	If ExistAtAll (RTH4)
		RightThighOrigin2=(RTH2+RTH3+RTH4)/3
		DummyRightFemur2 = [RightThighOrigin2,RightThighOrigin2-RTH3,RTH2-RTH4,yzx]
		$RTH1RelTech=RTH1/DummyRightFemur2
		PARAM($RTH1RelTech)
		
		RightThighOrigin3=(RTH1+RTH3+RTH4)/3
		DummyRightFemur3 = [RightThighOrigin3,RTH1-RTH3,RTH4-RightThighOrigin3,yzx]
		$RTH2RelTech=RTH2/DummyRightFemur3
		PARAM($RTH2RelTech)
		
		RightThighOrigin4=(RTH1+RTH2+RTH4)/3
		DummyRightFemur4 = [RightThighOrigin4,RTH1-RightThighOrigin4,RTH2-RTH4,yzx]
		$RTH3RelTech=RTH3/DummyRightFemur4
		PARAM($RTH3RelTech)
	EndIf
	
{*
Creates dummy thorax. VTR cluster should be placed so that VTR2 is where CLAV marker would usually be. VTR1 and VTR3 should be inferior to VTR2. Label right to left.
The position of VTR1-3 is also stored in the mp. They are recreated in the dynamic trials if one is missing (using VTR4).
Added by Dan Cottam, June 2016.
*}

	
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
	
{* 
Thorax pointer section. Creates virtual T10, sternum and C7 markers and stores their position relative to the dummy femur in the mp file. 
They are then recalled in the thorax section of the static and dynamic models.
*}	

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
	

{* =====================================================================================*}
{* 			   Start Dynamic Segmental Modelling 		    		*}
{* =====================================================================================*}

{* 					Head 				    		*}
{* =====================================================================================*}
{* use this replace macro if there are missing head markers, which cannot be interpolated *}
{* SUBSTITUTE4(LFHD, LBHD, RFHD, RBHD)*}

	If ExistAtAll (VTR2,C7)
		ThoraxOrigin = (C7+VTR2)/2
	ELSIF ExistAtAll (VC7)
		ThoraxOrigin = (VC7+VTR2)/2
	ELSE 
		ThoraxOrigin = (C7+CLAV)/2
	EndIf
	
	Height = $Height
	HeadOrigin = (LFHD+RFHD)/2
	LeftSideHead = (LFHD+LBHD)/2
	RightSideHead = (RFHD+RBHD)/2
	MidBackHead = (LBHD+RBHD)/2
	HeadMid = (LFHD+RFHD+LBHD+RBHD)/4
	HeadSize = DIST(HeadOrigin,MidBackHead)

	Head = [HeadMid,LeftSideHead-RightSideHead,HeadOrigin-MidBackHead,zyx]
	DrawSegment(Head)

	{*
	Vertex = {ThoraxOrigin(1), ThoraxOrigin(2), ThoraxOrigin(3)+((11.677/100)*Height)}
	$Vertex = Vertex/Head
	OUTPUT(Vertex,HeadOrigin,HeadMid)
	PARAM($Vertex)
	*}

{*================================================================*}
{*		           Pelvis	                          *}
{*================================================================*}

{* Pelvis and HJC's defined for femur replace macro in femur if required *}
{* Uses HJC location of Orthotrak (Shea et al.1997 Gait and Posture 5,157) *}
{* Even though this is essentially dynamic code, it is defined here so HJC can be used for femur REPLACE Macro to follow*}

{*Define Pelvis*}
	
	If EXISTATALL(LPSI,RPSI)
		SACR=(LPSI+RPSI)/2
	EndIf
	OUTPUT(SACR)

	PELOrigin=(LASI+RASI)/2
	MidPelvis=(SACR+PELOrigin)/2
	OUTPUT(PELOrigin,MidPelvis)

	Pelvis = [PELOrigin,RASI-LASI,SACR-PELOrigin,zyx]

{*Define HJC*}

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
	
	If $OptimalHip==2
	{* Stores SCoRE calculated HJC for use with dynamic UWA code, added by D Wells May 2017 *}
		
		InterASISDist=$InterASISdist
			If $InterASISdist == 0
				InterASISDist=DIST(LASI,RASI)
			EndIf
		aa = InterASISDist/2
		
		If ExistAtAll(Pelvis_RThigh_score)
			$Pelvis_RThigh_score = Pelvis_RThigh_score/Pelvis
			PARAM($Pelvis_RThigh_score)
			RHJC = $Pelvis_RThigh_score*Pelvis
		EndIf
		
		If ExistAtAll(Pelvis_LThigh_score)
			$Pelvis_LThigh_score = Pelvis_LThigh_score/Pelvis
			PARAM($Pelvis_LThigh_score)
			LHJC = $Pelvis_LThigh_score*Pelvis
		EndIf

		RHJC = $Pelvis_RThigh_score*Pelvis
		LHJC = $Pelvis_LThigh_score*Pelvis
		OUTPUT(LHJC,RHJC)

	EndIf
	
	$ThighLHJCRelTech=LHJC/DummyLeftFemur
	$ThighRHJCRelTech=RHJC/DummyRightFemur

	PARAM($ThighLHJCRelTech,$ThighRHJCRelTech)

	DrawSegment(Pelvis)

	{*HipJoints (not drawn)*}
	LHipJoint = LHJC+Attitude(Pelvis)
	RHipJoint = RHJC+Attitude(Pelvis)


{*================================================================*}
{*	Create ISB Global Coordinate System called Anatomy	  *}
{*================================================================*}
{* Align the positive direction of the global X-axis with the 
average direction of the sacrum to MidPelvis vector             *}
{*==============================================================*}

	avPEL = average((PELOrigin[7] + PELOrigin[6] + PELOrigin[5] + PELOrigin[4] + PELOrigin[3]
			 - PELOrigin[-7]- PELOrigin[-6]- PELOrigin[-5]- PELOrigin[-4]- PELOrigin[-3])/5)

	avTHOR = average((ThoraxOrigin[7] + ThoraxOrigin[6] + ThoraxOrigin[5] + ThoraxOrigin[4] + ThoraxOrigin[3]
			 - ThoraxOrigin[-7]- ThoraxOrigin[-6]- ThoraxOrigin[-5]- ThoraxOrigin[-4]- ThoraxOrigin[-3])/5)
	OUTPUT(avPEL)
	OUTPUT(avTHOR)

		If Exist(avPEL)
			XDOT = COMP(avPEL, {1, 0, 0})
			YDOT = COMP(avPEL, {0, 1, 0})
		else
			XDOT = COMP(avTHOR, {1, 0, 0})
			YDOT = COMP(avTHOR, {0, 1, 0})
		EndIf


		IF ABS(XDOT)>ABS(YDOT)
			Anatomy2ndDefLine = XDOT*{1, 0, 0}
		ELSE
			Anatomy2ndDefLine = YDOT*{0, 1, 0}
		ENDIF

	Anatomy = [{0, 0, 0}, {0, 0, 1}, Anatomy2ndDefLine, yzx]
	DrawSegment(Anatomy)

{* =============================================================================================*}
{* 									Trunk, Thorax & Torso				    		*}
{* =============================================================================================*}
{* use this replace macro if there are missing thorax or torso markers, which cannot be interpolated *}
{* USE WISELY!!*}
{*SUBSTITUTE4(STRN,CLAV,C7,T10)*}
{*SUBSTITUTE4(STRN,C7,RACR,T10)*}
{*SUBSTITUTE4(CLAV,C7,RACR,T10)*}
{*SUBSTITUTE4(RACR,CLAV,C7,T10)*}
{*SUBSTITUTE4(RACR,CLAV,C7,STRN)*}

{*
Changes made by Dan Cottam, June 2016:
	-Thorax re-defined to match upper body definition, which was correct. Old definition can be found in UWA legacy code mod file.
	-Trunk definition added. Definition copied from anthropometric section of lower body dynamic
	-LowerTorsoOrigin re-defined to MidHJC (from MidPelvis). Now matches de Leva's definitions (1996)
	-Section that erroneously re-defined the thorax at the end of this section has been removed
	-Allowance for virtual C7, T10 and STRN markers has been added
	-Trunk angle outputs have been altered in dynamic model to reflect changes here
*}

{*===================================Virtual Thorax===================================*}

{* 
Virtual T10 and STRN marker code initially written by Gill Weir, June 2014. Virtual C7 and optional statements added by Dan Cottam, June 2016
Reconstructs pointer defined marker positions by reading the PARAM values created after running the pointer macro in the pointer section above.
*}
	
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

{*Calculates Trunk Length. Allowance for VC7 added by Dan Cottam, June 2016*}

{*Average functions are not permitted in If statements. Therefore C7TrunkLength and VC7TrunkLength need to be calculated separately before being renamed as TrunkLength*}

	C7TrunkLength=Average(DIST(C7,MidHJC))
	VC7TrunkLength=Average(DIST(VC7,MidHJC))

	If ExistAtAll (VC7)
		$TrunkLength=VC7TrunkLength
	ELSE $TrunkLength=C7TrunkLength
	PARAM($TrunkLength)
	EndIf
	
{* Calculate Thorax Length *}

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

{*==============================================================*}
{*			    Femur				*}
{*==============================================================*}

{*
Femur pointer section. Creates virtual femoral condyle markers and stores their position in the local coordinate system created using the dummy femur in the mp file. 
They are then recalled in the femur section of the static and dynamic models. 
*}

If $Pointer==1

	{*Pointer Storage - Run pointer macro for femoral condyles*}
	
		If ExistAtAll(PLMFC,LeftThighOrigin) {*Pointer Left Medial Femoral Condyle*}
			Pointer(LeMFC,$LMFCpointerRelTech,PNTR1,PNTR2,PNTR3,PNTR4,PNTR5,PLMFC,DummyLeftFemur)
		EndIf

		If ExistAtAll(PLLFC,LeftThighOrigin) {*Pointer Left Lateral Femoral Condyle*}
			Pointer(LeLFC,$LLFCpointerRelTech,PNTR1,PNTR2,PNTR3,PNTR4,PNTR5,PLLFC,DummyLeftFemur)
		EndIf
		
		If ExistAtAll(PRMFC,RightThighOrigin) {*Pointer Right Medial Femoral Condyle*}
			Pointer(RiMFC,$RMFCpointerRelTech,PNTR1,PNTR2,PNTR3,PNTR4,PNTR5,PRMFC,DummyRightFemur)
		EndIf

		If ExistAtAll(PRLFC,RightThighOrigin) {*Pointer Right Lateral Femoral Condyle*}
			Pointer(RiLFC,$RLFCpointerRelTech,PNTR1,PNTR2,PNTR3,PNTR4,PNTR5,PRLFC,DummyRightFemur)
		EndIf

	{*Pointer Output - Pointer defined epicondyle markers*}

		LeMFC= $LMFCpointerRelTech*DummyLeftFemur
		OUTPUT(LeMFC)
		
		LeLFC= $LLFCpointerRelTech*DummyLeftFemur
		OUTPUT(LeLFC)

		RiMFC=$RMFCpointerRelTech*DummyRightFemur
		OUTPUT(RiMFC)			

		RiLFC=$RLFCpointerRelTech*DummyRightFemur
		OUTPUT(RiLFC)

EndIf		

{* Output pointer defined anatomical points relative to dummy segment. Runs if pointers have been used for the femoral condyles *}

If ExistAtAll($RLFCpointerRelTech,$RMFCpointerRelTech)
	RiLFC=$RLFCpointerRelTech*DummyRightFemur
	RiMFC=$RMFCpointerRelTech*DummyRightFemur
	OUTPUT(RiMFC,RiLFC)
EndIf

If ExistAtAll($LLFCpointerRelTech,$LMFCpointerRelTech)
	LeLFC=$LLFCpointerRelTech*DummyLeftFemur
	LeMFC=$LMFCpointerRelTech*DummyLeftFemur
	OUTPUT(LeMFC,LeLFC)
EndIf
	
	
{* 
Stores femoral condyle positions in mp file (in dummy femur LCS). Runs if anatomical markers have been used for the femoral condyles. 
These markers are recreated in the dynamic model. 
*}

If $Pointer==0	

	If ExistAtAll(RMFC,RightThighOrigin)
		$RMFCAnatRelTech=RMFC/DummyRightFemur
	EndIf		
	PARAM($RMFCAnatRelTech)

	If ExistAtAll(RLFC,RightThighOrigin)
		$RLFCAnatRelTech=RLFC/DummyRightFemur
	EndIf
	PARAM($RLFCAnatRelTech)

	If ExistAtAll(LMFC,LeftThighOrigin)	
		$LMFCAnatRelTech=LMFC/DummyLeftFemur
	EndIf
	PARAM($LMFCAnatRelTech)
	
	If ExistAtAll(LLFC,LeftThighOrigin)
		$LLFCAnatRelTech=LLFC/DummyLeftFemur
	EndIf
	PARAM($LLFCAnatRelTech)

EndIf

{* Output anatomically defined femoral condyles. Only really needed if the physical condyle markers are not present in all static trials *}	

	If ExistAtAll($LMFCAnatRelTech,$LLFCAnatRelTech)
		LMFC=$LMFCAnatRelTech*DummyLeftFemur
		LLFC=$LLFCAnatRelTech*DummyLeftFemur
		OUTPUT(LMFC,LLFC)
	EndIf

	If ExistAtAll($RMFCAnatRelTech,$RLFCAnatRelTech)
		RMFC=$RMFCAnatRelTech*DummyRightFemur
		RLFC=$RLFCAnatRelTech*DummyRightFemur
		OUTPUT(RMFC,RLFC)
	EndIf

	
{* Define Knee Joint Centres and femur segment*}

	If ExistAtAll(LeLFC,LeMFC)
		LKJC=((LeLFC+LeMFC)/2)
		LeftFemur=[LKJC,LHJC-LKJC,LeLFC-LeMFC,yxz]
	Else	
		LKJC=((LLFC+LMFC)/2)
		LeftFemur=[LKJC,LHJC-LKJC,LLFC-LMFC,yxz]
	EndIf
	OUTPUT(LKJC)

	If ExistAtAll(RiLFC,RiMFC)
		RKJC=((RiLFC+RiMFC)/2)
		RightFemur=[RKJC,RHJC-RKJC,RiMFC-RiLFC,yxz]
	Else
		RKJC=((RLFC+RMFC)/2)
		RightFemur=[RKJC,RHJC-RKJC,RMFC-RLFC,yxz]		
	EndIf
	OUTPUT(RKJC)

{*Helical Knee Axis Section*}	
	
	If $HelicalKnee == 1
		LKHel1=$LKHA1*DummyLeftFemur
		LKHel2=$LKHA2*DummyLeftFemur  
		RKHel1=$RKHA1*DummyRightFemur
		RKHel2=$RKHA2*DummyRightFemur
		OUTPUT(LKHel1,LKHel2,RKHel1,RKHel2)

		LKJC=PERP(LKJC,LKHel1,LKHel2)
		RKJC=PERP(RKJC,RKHel1,RKHel2) 
		OUTPUT(LKJC,RKJC)
		
	EndIf
	
	If $HelicalKnee == 2
	{* Stores SCoRE/SARA calculated KJC + FE axis of rotation for use with dynamic UWA code, added by D Wells May 2017 *}

		If ExistAtAll(RThigh_RTibia_score)
		
			$RKHA1 = RThigh_RTibia_score/DummyRightFemur
			$RKHA2 = RThigh_RTibia_sara/DummyRightFemur
			PARAM($RKHA1,$RKHA2)

		EndIf	

		If ExistAtAll(LThigh_LTibia_score)
		
			$LKHA1 = LThigh_LTibia_score/DummyLeftFemur
			$LKHA2 = LThigh_LTibia_sara/DummyLeftFemur
			PARAM($LKHA1,$LKHA2)

		EndIf	
		
		LKHel1=$LKHA1*DummyLeftFemur
		LKHel2=$LKHA2*DummyLeftFemur  
		RKHel1=$RKHA1*DummyRightFemur
		RKHel2=$RKHA2*DummyRightFemur
		OUTPUT(LKHel1,LKHel2,RKHel1,RKHel2)
		
		LKJC=PERP(LKJC,LKHel1,LKHel2)
		RKJC=PERP(RKJC,RKHel1,RKHel2)
		
		OUTPUT(LKJC,RKJC)
		
	EndIf
	
{* check to make sure the helical axes vectors are in the right direction *}
	
	If ExistAtAll(LKHel1,RKHel2)
	
		If ExistAtAll(LMFC,LLFC)	
			check1=comp(LKHel2-LKHel1,LMFC-LLFC)
				If check1 < 0
					LeftFemur=[LKJC,LHJC-LKJC,LKHel2-LKHel1,yxz]
				Else 
					LeftFemur=[LKJC,LHJC-LKJC,LKHel1-LKHel2,yxz]
				EndIf
		EndIf
		
		If ExistAtAll(LeMFC,LeLFC)	
			check1=comp(LKHel2-LKHel1,LeMFC-LeLFC)
				If check1 < 0
					LeftFemur=[LKJC,LHJC-LKJC,LKHel2-LKHel1,yxz]
				Else 
					LeftFemur=[LKJC,LHJC-LKJC,LKHel1-LKHel2,yxz]
				EndIf
		EndIf

		If ExistAtAll(RMFC,RLFC)
			check2=comp(RKHel2-RKHel1,RMFC-RLFC)
			If check2 < 0
				RightFemur=[RKJC,RHJC-RKJC,RKHel1-RKHel2,yxz]
			Else 
				RightFemur=[RKJC,RHJC-RKJC,RKHel2-RKHel1,yxz]		
			EndIf
		EndIf
		
		If ExistAtAll(RiMFC,RiLFC)
			check2=comp(RKHel2-RKHel1,RiMFC-RiLFC)
			If check2 < 0
				RightFemur=[RKJC,RHJC-RKJC,RKHel1-RKHel2,yxz]
			Else 
				RightFemur=[RKJC,RHJC-RKJC,RKHel2-RKHel1,yxz]		
			EndIf
		EndIf
	EndIf
	
	DrawSegment(LeftFemur)
	DrawSegment(RightFemur)

{*================================================================*}
{* 					Tibia 					*}
{*================================================================*}

{*Create dummy tibia segments. Fourth marker added June 2016.*}

{*Create dummy left tibia and store position of LTB1-3 if fourth marker is being used. *}

	LeftTibOrigin = (LTB1+LTB2+LTB3)/3
	DummyLeftTibia = [LeftTibOrigin,LTB1-LTB3,LTB2-LeftTibOrigin,yzx]
	OUTPUT(LeftTibOrigin)
	
	If ExistAtAll(LTB4)
		LeftTibOrigin2 = (LTB2+LTB3+LTB4)/3
		DummyLeftTibia2 = [LeftTibOrigin2,LeftTibOrigin2-LTB3,LTB2-LTB4,yzx]
		$LTB1RelTech= LTB1/DummyLeftTibia2
		PARAM($LTB1RelTech)
		
		LeftTibOrigin3 = (LTB1+LTB3+LTB4)/3
		DummyLeftTibia3 = [LeftTibOrigin3,LTB1-LTB3,LeftTibOrigin3-LTB4,yzx]
		$LTB2RelTech=LTB2/DummyLeftTibia3
		PARAM($LTB2RelTech)
		
		LeftTibOrigin4 = (LTB1+LTB2+LTB4)/3
		DummyLeftTibia4 = [LeftTibOrigin4,LTB1-LeftTibOrigin4,LTB2-LTB4,yzx]
		$LTB3RelTech=LTB3/DummyLeftTibia4
		PARAM($LTB3RelTech)
	EndIf

{*Create dummy right tibia segment and store position of RTB1-3 if fourth marker is being used*}

	RightTibOrigin =(RTB1+RTB2+RTB3)/3
	DummyRightTibia = [RightTibOrigin,RTB1-RTB3,RTB2-RightTibOrigin,yzx]
	OUTPUT(RightTibOrigin)
	
	If ExistAtAll(RTB4)
		RightTibOrigin2 = (RTB2+RTB3+RTB4)/3
		DummyRightTibia2 = [RightTibOrigin2,RightTibOrigin2-RTB3,RTB2-RTB4,yzx]
		$RTB1RelTech= RTB1/DummyRightTibia2
		PARAM($RTB1RelTech)
		
		RightTibOrigin3 = (RTB1+RTB3+RTB4)/3
		DummyRightTibia3 = [RightTibOrigin3,RTB1-RTB3,RightTibOrigin3-RTB4,yzx]
		$RTB2RelTech=RTB2/DummyRightTibia3
		PARAM($RTB2RelTech)
		
		RightTibOrigin4 = (RTB1+RTB2+RTB4)/3
		DummyRightTibia4 = [RightTibOrigin4,RTB1-RightTibOrigin4,RTB2-RTB4,yzx]
		$RTB3RelTech=RTB3/DummyRightTibia4
		PARAM($RTB3RelTech)
	EndIf

{*Store Left malleoli marker positions in mp file*}

	If ExistAtAll(LMMAL,LLMAL)
		$LMMALAnatRelTech = LMMAL/DummyLeftTibia 
		$LLMALAnatRelTech = LLMAL/DummyLeftTibia 
	PARAM($LMMALAnatRelTech, $LLMALAnatRelTech)
	EndIf	

{*Store Right malleoli marker positions in mp file*}
	
	If ExistAtAll(RMMAL,RLMAL)
		$RMMALAnatRelTech = RMMAL/DummyRightTibia 
		$RLMALAnatRelTech = RLMAL/DummyRightTibia 
	PARAM($RMMALAnatRelTech, $RLMALAnatRelTech)
	EndIf

{*Recreate malleoli markers. Only really needed if physical markers are not present in all static trials*}

	RMMAL=$RMMALAnatRelTech*DummyRightTibia
	RLMAL=$RLMALAnatRelTech*DummyRightTibia
	LMMAL=$LMMALAnatRelTech*DummyLeftTibia
	LLMAL=$LLMALAnatRelTech*DummyLeftTibia
	OUTPUT(LMMAL,LLMAL,RMMAL,RLMAL)
	
{* Output ankle joint centres*}

	If ExistAtAll (LMMAL,LLMAL)
		LAJC = ((LMMAL+LLMAL)/2)
	OUTPUT(LAJC)
	EndIf

	If ExistAtAll (RMMAL,RLMAL)
		RAJC = ((RMMAL+RLMAL)/2)
	OUTPUT(RAJC)
	EndIf

{* Define Right Tibia *}

	RightTibia=[RAJC,RKJC-RAJC,RMMAL-RLMAL,yxz]
	DrawSegment(RightTibia)

	LeftTibia=[LAJC,LKJC-LAJC,LLMAL-LMMAL,yxz]
	DrawSegment(LeftTibia)

{* Find femur, tibia and leg lengths and output for use in the dynamic knee model *}

	LFemurLengthm = DIST(LKJC,LHJC)/1000
	RFemurLengthm = DIST(RKJC,RHJC)/1000
	LTibiaLengthm = DIST(LAJC,LKJC)/1000
	RTibiaLengthm = DIST(RAJC,RKJC)/1000
	LLegLengthm = LFemurLengthm+LTibiaLengthm
	RLegLengthm = RFemurLengthm+RTibiaLengthm
	PARAM(LFemurLengthm,RFemurLengthm,LTibiaLengthm,RTibiaLengthm,LLegLengthm,RLegLengthm)


{* ==============================================================================================*}
{* 					Foot               					 *}
{* ==============================================================================================*}

{* Each foot has three real (L/RCAL,L/RMT1,L/RMT5) markers and one virtual (L/RFTML) marker *}

	LFTML=(LMT5+LMT1)/2
	RFTML=(RMT5+RMT1)/2
	LeftFootOrigin=LFTML
	RightFootOrigin=RFTML

{*define left foot segment with lateral(z) axis along (LMT1-LMT5)*}
	
	DummyLeftFoot = [LCAL,LeftFootOrigin-LCAL,LMT1-LMT5,xyz]
	
{*define right foot segment with lateral(z) axis along (RMT5-RMT1)*}

	DummyRightFoot = [RCAL,RightFootOrigin-RCAL,RMT5-RMT1,xyz]

{* define foot length for the drawing section and place this distance into the parameter file *}

	RFootLength = DIST(RCAL,RMT1)
	LFootLength = DIST(LCAL,LMT1)

	$LFemurScale = DIST(LKJC,LHJC)/100
	$RFemurScale = DIST(RKJC,RHJC)/100  
	$LTibiaScale=DIST(LKJC,LAJC)/100
	$RTibiaScale=DIST(RKJC,RAJC)/100
	$LFootScale=$LFootLength/100
	$RFootScale=$RFootLength/100
	
	PARAM($LFemurScale,$RFemurScale,$LTibiaScale,$RTibiaScale,$LFootScale,$RFootScale)

{*=========================*}
{*   Foot Calibration Rig   *}
{*=========================*}

{* the calibration rig is defined such that its axes align with the axes
 of the other body segments. *}

	If ExistAtAll(RIGR,RIGL) 
		CalRig = [(RIGR+RIGL)/2,RIGA-RIGP,RIGR-RIGL,xyz]
	Else	
		{* When Foot Rig is not used *}
		CalRig = Anatomy
	EndIf
	DrawSegment(CalRig)
	
{*==============================================================*}
{* Start of the Drawing Section *}
{*==============================================================*}

{*Build Pelvis*}

	Pel = [PELOrigin,LASI-RASI,PELOrigin-SACR,yzx]

	bb = aa/2
	CX = bb*{-3,0,-1.8}*Pel
	LSI = bb*{-3.5,0.5,0}*Pel
	RSI = bb*{-3.5,-0.5,0}*Pel
	LIC = bb*{-1.7,1.5,0.7}*Pel
	RIC = bb*{-1.7,-1.5,0.7}*Pel
	LIS = bb*{-0.5,2.1,0}*Pel
	RIS = bb*{-0.5,-2.1,0}*Pel
	LIT = bb*{-1,1,-1.8}*Pel
	RIT = bb*{-1,-1,-1.8}*Pel
	LIP = bb*{-1.1,1,-1}*Pel
	RIP = bb*{-1.1,-1,-1}*Pel
	SP = bb*{-0.5,0,-1.5}*Pel
	OUTPUT(CX,LSI,RSI,LIC,RIC,LIS,RIS,LIT,RIT,SP,LIP,RIP)

{*Draw LFemur, using 16 vertices, scaled to bone length*}	

	If ExistAtAll(LLFC,LMFC)
		LFemur=[LKJC,LHJC-LKJC,LLFC-LMFC,zxy]
	EndIf
	
	If ExistAtAll(LeLFC,LeMFC)
		LFemur=[LKJC,LHJC-LKJC,LeLFC-LeMFC,zxy]
	EndIf

	LFEM1 = $LFemurScale*{2,0,100}*LFemur
	LFEM2 = $LFemurScale*{0,-1,98}*LFemur
	LFEM3 = $LFemurScale*{-2,0,100}*LFemur
	LFEM4 = $LFemurScale*{0,1,102}*LFemur
	LFEM5 = $LFemurScale*{3,4,87}*LFemur
	LFEM6 = $LFemurScale*{0,1,87}*LFemur
	LFEM7 = $LFemurScale*{-3,4,87}*LFemur
	LFEM8 = $LFemurScale*{0,10,94}*LFemur
	LFEM9 = $LFemurScale*{3,0,12}*LFemur
	LFEM10 = $LFemurScale*{0,-3,12}*LFemur
	LFEM11 = $LFemurScale*{-3,0,12}*LFemur
	LFEM12 = $LFemurScale*{0,3,12}*LFemur
	LFEM13 = $LFemurScale*{5,0,0}*LFemur
	LFEM14 = $LFemurScale*{0,-8,0}*LFemur
	LFEM15 = $LFemurScale*{-10,0,8}*LFemur
	LFEM16 = $LFemurScale*{0,7,0}*LFemur

	OUTPUT(LFEM1,LFEM2,LFEM3,LFEM4,LFEM5,LFEM6,LFEM7,LFEM8)
	OUTPUT(LFEM9,LFEM10,LFEM11,LFEM12,LFEM13,LFEM14,LFEM15,LFEM16)

{*Draw RFemur, using 16 vertices, scaled to bone length*}	
	
	If ExistAtAll(RLFC,RMFC)
		RFemur=[RKJC,RHJC-RKJC,RMFC-RLFC,zxy]
	EndIf

	If ExistAtAll(RiLFC,RiMFC)
		RFemur=[RKJC,RHJC-RKJC,RiMFC-RiLFC,zxy]
	EndIf

	RFEM1 = $RFemurScale*{2,0,100}*RFemur
	RFEM2 = $RFemurScale*{0,1,98}*RFemur
	RFEM3 = $RFemurScale*{-2,0,100}*RFemur
	RFEM4 = $RFemurScale*{0,1,102}*RFemur
	RFEM5 = $RFemurScale*{3,-4,87}*RFemur
	RFEM6 = $RFemurScale*{0,-1,87}*RFemur
	RFEM7 = $RFemurScale*{-3,-4,87}*RFemur
	RFEM8 = $RFemurScale*{0,-10,94}*RFemur
	RFEM9 = $RFemurScale*{3,0,12}*RFemur
	RFEM10 = $RFemurScale*{0,3,12}*RFemur
	RFEM11 = $RFemurScale*{-3,0,12}*RFemur
	RFEM12 = $RFemurScale*{0,-3,12}*RFemur
	RFEM13 = $RFemurScale*{5,0,0}*RFemur
	RFEM14 = $RFemurScale*{0,8,0}*RFemur
	RFEM15 = $RFemurScale*{-10,0,8}*RFemur
	RFEM16 = $RFemurScale*{0,-7,0}*RFemur

	OUTPUT(RFEM1,RFEM2,RFEM3,RFEM4,RFEM5,RFEM6,RFEM7,RFEM8)
	OUTPUT(RFEM9,RFEM10,RFEM11,RFEM12,RFEM13,RFEM14,RFEM15,RFEM16)

{*Draw LTibia, using 16 vertices, scaled to bone length*}	
	
	LTibia=[LAJC,LKJC-LAJC,LLMAL-LMMAL,zxy]

	LTIB1 = $LTibiaScale*{8,0,98}*LTibia
	LTIB2 = $LTibiaScale*{-4,-10,100}*LTibia
	LTIB3 = $LTibiaScale*{-4,0,100}*LTibia
	LTIB4 = $LTibiaScale*{-4,10,100}*LTibia
	LTIB5 = $LTibiaScale*{3,0,80}*LTibia
	LTIB6 = $LTibiaScale*{0,-3,80}*LTibia
	LTIB7 = $LTibiaScale*{-3,0,80}*LTibia
	LTIB8 = $LTibiaScale*{0,5,80}*LTibia
	LTIB9 = $LTibiaScale*{3,0,20}*LTibia
	LTIB10 = $LTibiaScale*{0,-3,20}*LTibia
	LTIB11 = $LTibiaScale*{-3,0,20}*LTibia
	LTIB12 = $LTibiaScale*{0,6,20}*LTibia
	LTIB13 = $LTibiaScale*{10,0,10}*LTibia
	LTIB14 = $LTibiaScale*{0,-10,4}*LTibia
	LTIB15 = $LTibiaScale*{-5,0,10}*LTibia
	LTIB16 = $LTibiaScale*{0,10,0}*LTibia

	OUTPUT(LTIB1,LTIB2,LTIB3,LTIB4,LTIB5,LTIB6,LTIB7,LTIB8)
	OUTPUT(LTIB9,LTIB10,LTIB11,LTIB12,LTIB13,LTIB14,LTIB15,LTIB16)

{*Draw RTibia, using 16 vertices, scaled to bone length*}
	
	RTibia=[RAJC,RKJC-RAJC,RMMAL-RLMAL,zxy]

	RTIB1 = $RTibiaScale*{8,0,98}*RTibia
	RTIB2 = $RTibiaScale*{-4,10,100}*RTibia
	RTIB3 = $RTibiaScale*{-4,0,100}*RTibia
	RTIB4 = $RTibiaScale*{-4,-10,100}*RTibia
	RTIB5 = $RTibiaScale*{3,0,80}*RTibia
	RTIB6 = $RTibiaScale*{0,3,80}*RTibia
	RTIB7 = $RTibiaScale*{-3,0,80}*RTibia
	RTIB8 = $RTibiaScale*{0,-5,80}*RTibia
	RTIB9 = $RTibiaScale*{3,0,20}*RTibia
	RTIB10 = $RTibiaScale*{0,3,20}*RTibia
	RTIB11 = $RTibiaScale*{-3,0,20}*RTibia
	RTIB12 = $RTibiaScale*{0,-6,20}*RTibia
	RTIB13 = $RTibiaScale*{10,0,10}*RTibia
	RTIB14 = $RTibiaScale*{0,10,4}*RTibia
	RTIB15 = $RTibiaScale*{-5,0,10}*RTibia
	RTIB16 = $RTibiaScale*{0,-10,0}*RTibia

	OUTPUT(RTIB1,RTIB2,RTIB3,RTIB4,RTIB5,RTIB6,RTIB7,RTIB8)
	OUTPUT(RTIB9,RTIB10,RTIB11,RTIB12,RTIB13,RTIB14,RTIB15,RTIB16)

{*Draw LFoot, using 6 vertices, scaled to bone length*}	
	
	LFoot = [LeftFootOrigin,LeftFootOrigin-LCAL,LMT1-LMT5,xzy]

	LFOO1 = $LFootScale*{25,-25,-20}*LFoot
	LFOO2 = $LFootScale*{5,20,-20}*LFoot
	LFOO3 = $LFootScale*{-70,10,-20}*LFoot
	LFOO4 = $LFootScale*{-70,-10,-20}*LFoot
	LFOO5 = $LFootScale*{-70,0,20}*LFoot
	LFOO6 = $LFootScale*{-50,0,20}*LFoot
	
	OUTPUT(LFOO1,LFOO2,LFOO3,LFOO4,LFOO5,LFOO6)

{*Draw RFoot, using 6 vertices, scaled to bone length*}	

	RFoot = [RightFootOrigin,RightFootOrigin-RCAL,RMT5-RMT1,xzy]

	RFOO1 = $RFootScale*{25,25,-20}*RFoot
	RFOO2 = $RFootScale*{5,-20,-20}*RFoot
	RFOO3 = $RFootScale*{-70,-10,-20}*RFoot
	RFOO4 = $RFootScale*{-70,10,-20}*RFoot
	RFOO5 = $RFootScale*{-70,0,20}*RFoot
	RFOO6 = $RFootScale*{-50,0,20}*RFoot

	OUTPUT(RFOO1,RFOO2,RFOO3,RFOO4,RFOO5,RFOO6)

{*==============================================================*}	
{*Angle Outputs*}
{*==============================================================*}

	TrunkAngleGlob = <Anatomy, Trunk, zxy>
	TrunkAngleGlob = <-1(TrunkAngleGlob),2(TrunkAngleGlob),3(TrunkAngleGlob)>
	TrunkAngleBaker= <Anatomy, Trunk, yxz>
	TrunkAngleBaker = <1(TrunkAngleBaker),2(TrunkAngleBaker),-3(TrunkAngleBaker)>
	ThoraxAngle 	= -<Anatomy, Thorax, zxy>
	ThoraxAngle = <-1(ThoraxAngle),2(ThoraxAngle),3(ThoraxAngle)>
	ThoraxAngleBaker = -<Anatomy, Thorax, yxz>
	ThoraxAngleBaker = <1(ThoraxAngleBaker),2(ThoraxAngleBaker),-3(ThoraxAngleBaker)>
	TorsoAngle 	= -<Anatomy, Torso, zxy>
	TorsoAngle = <-1(TorsoAngle),2(TorsoAngle),3(TorsoAngle)>
	TorsoAngleBaker = -<Anatomy, Torso, yxz>
	TorsoAngleBaker = <1(TorsoAngleBaker),2(TorsoAngleBaker),-3(TorsoAngleBaker)>
	ThortoPelAngle = -<Pelvis,Thorax,zxy>
	ThortoPelAngle = <-1(ThortoPelAngle),2(ThortoPelAngle),3(ThortoPelAngle)>
	
	LHipAngle = -<Pelvis,LeftFemur,zxy>
	LHipAngle = <1(LHipAngle),-2(LHipAngle),-3(LHipAngle)> 
	RHipAngle = -<Pelvis,RightFemur,zxy>
	RHipAngle = <1(RHipAngle),2(RHipAngle),3(RHipAngle)> 

	LKneeAngle = -<LeftFemur,LeftTibia,zxy>
	LKneeAngle = <-1(LKneeAngle),-2(LKneeAngle),-3(LKneeAngle)> 
	RKneeAngle = -<RightFemur,RightTibia,zxy>
	RKneeAngle = <-1(RKneeAngle),2(RKneeAngle),3(RKneeAngle)>

	LAnkleAngle = -<LeftTibia,DummyLeftFoot,zxy>
	LAnkleAngle = <1(LAnkleAngle),-2(LAnkleAngle),-3(LAnkleAngle)>
	RAnkleAngle = -<RightTibia,DummyRightFoot,zxy>
	RAnkleAngle = <1(RAnkleAngle),2(RAnkleAngle),3(RAnkleAngle)>

	PelvisAngle = -<Anatomy, Pelvis, zxy>
	PelvisAngle = <-1(PelvisAngle),2(PelvisAngle),-3(PelvisAngle)>

	OUTPUT(LHipAngle,RHipAngle,LKneeAngle,RKneeAngle,LAnkleAngle,RAnkleAngle,PelvisAngle)
	OUTPUT(TrunkAngleGlob,TrunkAngleBaker,ThoraxAngle,ThoraxAngleBaker,TorsoAngle,TorsoAngleBaker,ThortoPelAngle)

{* Thorax Angle Static Offsets --> to check position of the thorax relative to the pelvis during static trials *}

	$ThortoPelAngleFE = ThortoPelAngle(1)
	$ThortoPelAngleLF = ThortoPelAngle(2)
	$ThortoPelAngleRT = ThortoPelAngle(3)
	PARAM($ThortoPelAngleFE,$ThortoPelAngleLF,$ThortoPelAngleRT)	

{* Define joint flexion/extension offsets and write to static c3dfile *}

	OffsetPelvis = AVERAGE(PelvisAngle)	
	OffsetLHip = AVERAGE(LHipAngle)
	OffsetRHip = AVERAGE(RHipAngle)
	OffsetLKnee = AVERAGE(LKneeAngle)
	OffsetRKnee = AVERAGE(RKneeAngle)
	OffsetLAnkle = AVERAGE(LAnkleAngle)
	OffsetRAnkle = AVERAGE(RAnkleAngle)

	OUTPUT(OffsetLHip,OffsetRHip,OffsetLKnee,OffsetRKnee,OffsetLAnkle,OffsetRAnkle)

{* find rotation offset of ankle FE axis relative to the knee FE axis *}

	$LTibRotOffset = LKneeAngle(3)
	$RTibRotOffset = RKneeAngle(3)
	PARAM($LTibRotOffset,$RTibRotOffset)

{* 
Using the foot progression angle and eversion angle, calculate virtual
markers relative to the feet segments. These virtual markers are used in 
HMGait.mod to properly reconstruct the foot segment 
*}

	RFootMidline = RCAL+100*1(CalRig)
	LFootMidline = LCAL+100*1(CalRig)
	LFootSide = LCAL+100*3(CalRig)
	RFootSide = RCAL+100*3(CalRig)

 	LeftFoot = [LCAL,LFootMidline-LCAL,LFootSide-LCAL,xyz]
	RightFoot = [RCAL,RFootMidline-RCAL,RFootSide-RCAL,xyz]

 	LeftFoot = ROT(LeftFoot,2(LeftFoot),$LFootAbduction)   
 	LeftFoot = ROT(LeftFoot,1(LeftFoot),$LFootEversion)   

 	RightFoot = ROT(RightFoot,2(RightFoot),-$RFootAbduction) 
 	RightFoot = ROT(RightFoot,1(RightFoot),-$RFootEversion) 

 	DrawSegment(RightFoot)
 	DrawSegment(LeftFoot)
	
	{* create a set of new points to define the actual foot segment *}
	
	 newLfootmid = LCAL+100*1(LeftFoot)
	 newRfootmid = RCAL+100*1(RightFoot)
	 newLfootside = LCAL+100*3(LeftFoot)
	 newRfootside = RCAL+100*3(RightFoot)
	 OUTPUT(newRfootmid, newLfootmid,newLfootside,newRfootside)

	{* save the relationship of the new points relative to the dummy foot so that the 
	   real foot can be recreated in HMGait.mod  *}

	 $LFMIDvirtual = newLfootmid/DummyLeftFoot
	 $RFMIDvirtual = newRfootmid/DummyRightFoot
	 $LFSIDEvirtual = newLfootside/DummyLeftFoot
	 $RFSIDEvirtual = newRfootside/DummyRightFoot
 
 	PARAM($LFMIDvirtual,$RFMIDvirtual,$LFSIDEvirtual,$RFSIDEvirtual) 
	
{*	
LowerBodyModelVer = {10,08,2016}
OUTPUT(LowerBodyModelVer)
$LowerBodyModelVer = 10082016
PARAM($LowerBodyModelVer)
*}
{* End of processing *}



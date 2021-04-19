{*VICON BodyLanguage (tm)*}

{*====================================================================================================================================*}
{*====================================================UWA LOWER BODY DYNAMIC MODEL====================================================*}
{*====================================================================================================================================*}

{*
	1. Program-Name Creation-Date
	UWALBDynamic.mod 30/June/2016
	2. Original-Author (Email-Address)
	Multiple Contributors
	3. Last-Updated-By (Email-Address)
	Dan Cottam (daniel.cottam@research.uwa.edu.au)
	4. Short-Description
	The generic UWA lower body dynamic bodybuilder model
	5. Notes
	Please see documents on Sharepoint named "Changes to UWA Lower and Upper Body Models (June, 2016)" and Version 1.1 UWA Model Changes (August 2016) for information on model changes.
	To use SCoRE/SARA functional hip and knee definitions enter '2' into Subject Parameters 'OptimalHip' and 'HelicalKnee' (D Wells May 2017)
	6. Modification-History
	Version  Author     Date                 Change
	1.1       DC    10/August/2016     See Version 1.1 UWA Model Changes on Sharepoint
*}

{*Start of macro section*}
{*==============================================================*}

{* ============ MACRO SUBSTITUTE4 ============*}
macro SUBSTITUTE4(p1,p2,p3,p4)
{*Replaces any point missing from set of four fixed in a segment *}
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
Output(O#segm,X#segm,Y#segm,Z#segm)
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
Point#LAcc=((Point[-2]-(8*Point[-1])+(8*Point[1])-Point[2])/(12*FrameTimeLength))
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
changing the signs of the joint Moment to clinical/functionally relevant conventions *}

{* Transform moment into parent coordinate system *}
ChildAttitude = ATTITUDE(child)
ParentAttitude = ATTITUDE(parent)
MomentInGlobal = Joint#Moment*ChildAttitude
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

{*======================*}
Macro CreateRelPoint(PGlob,segm,NewPointName)
{*Creates a new point relative to a local segment*}
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

{* Head, Shoulder and Thorax Markers*}

	OptionalPoints(LFHD, RFHD, LBHD, RBHD)
	OptionalPoints(C7, T10, CLAV, STRN)
	OptionalPoints(VTR1, VTR2, VTR3, VTR4)
	OptionalPoints(LACR, LASH, LPSH) 
	OptionalPoints(RACR, RASH, RPSH)

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
	
{*Ball Markers*}

	OptionalPoints(Ball1, Ball2, Ball3, Ball4)
	
{*Hockey Stick Markers*}
	
	OptionalPoints(Stick1,Stick2,Stick3)
	OptionalPoints($StickTopRelTech,$StickPostRelTech,$StickInfRelTech,$StickAntRelTech)

{*Virtual Markers*}

	OptionalPoints($LOptimalHip,$ROptimalHip)
	OptionalPoints($LKHA1,$LKHA2,$RKHA1,$RKHA2)
	OptionalPoints($LMFCAnatRelTech,$LLFCAnatRelTech,$RMFCAnatRelTech,$RLFCAnatRelTech)
	OptionalPoints($LMFCPointerRelTech,$LLFCPointerRelTech,$RMFCPointerRelTech,$RLFCPointerRelTech)
	OptionalPoints($LMMALAnatRelTech,$LLMALAnatRelTech,$RMMALAnatRelTech,$RLMALAnatRelTech)
	OptionalPoints($LFMIDvirtual,$RFMIDvirtual)
	OptionalPoints($LFSIDEvirtual,$RFSIDEvirtual)
	OptionalPoints($VTR1RelTech,$VTR2RelTech,$VTR3RelTech)
	OptionalPoints($VT10pointerRelTech,$VSTRNpointerRelTech,$VC7pointerRelTech)
	OptionalPoints($RTH1RelTech,$RTH2RelTech,$RTH3RelTech)
	OptionalPoints($LTH1RelTech,$LTH2RelTech,$LTH3RelTech)
	OptionalPoints($LTB1RelTech,$LTB2RelTech,$LTB3RelTech)
	OptionalPoints($RTB1RelTech,$RTB2RelTech,$RTB3RelTech)
	
	OptionalReactions(ForcePlate1, ForcePlate2, ForcePlate3)

{* This section is for foot code only, please comment out if you do not want these points to be optional*}
	
	OptionalPoints($ThighLHJCRelTech, $ThighRHJCRelTech)
	OptionalNumbers($LTibRotOffset, $RTibRotOffset)
	OptionalNumbers($LFemurScale, $RFemurScale)
	OptionalNumbers($LTibiaScale, $RTibiaScale)
	
{* SCoRE/SARA Markers*}
	
	OptionalPoints($Pelvis_LThigh_score, $Pelvis_RThigh_score)	
	
DrawGlobal(200)

{*==============================================================*}
{* START Initialisations*}
{*==============================================================*}

ForceThreshold = 15
{* Show the force vectors *}

if EXIST(ForcePlate1)
	Force1 = ForcePlate1(1)
	Moment1 = ForcePlate1(2)
	Centre1 = ForcePlate1(3)
	if (ABS ( Force1 ) > ForceThreshold)
		Point1 = Centre1 + { -Moment1(2)/Force1(3), Moment1(1)/Force1(3), -Centre1(3) }
	else
		Point1 = Centre1
	endif
	ForceVector1 = Point1 + Force1
	OUTPUT(Point1,ForceVector1) 
endif

if EXIST(ForcePlate2)
	Force2 = ForcePlate2(1)
	Moment2 = ForcePlate2(2)
	Centre2 = ForcePlate2(3)
	if ( ABS ( Force2 ) > ForceThreshold)
		Point2 = Centre2 + { -Moment2(2)/Force2(3), Moment2(1)/Force2(3), -Centre2(3) }
	else 
		Point2 = Centre2
	endif
	ForceVector2 = Point2 + Force2
	OUTPUT(Point2,ForceVector2)
endif

if EXIST(ForcePlate3)
	Force3 = ForcePlate3(1)
	Moment3 = ForcePlate3(2)
	Centre3 = ForcePlate3(3)
	if ( ABS ( Force3 ) > ForceThreshold)
		Point3 = Centre3 + { -Moment3(2)/Force3(3), Moment3(1)/Force3(3), -Centre3(3) }
	else 
		Point3 = Centre3
	endif
	ForceVector3 = Point3 + Force3
	OUTPUT(Point3,ForceVector3)
endif


{*=============================================================================*}
{* Anthropometry Values *}

{* Add the hierarchy, mass, and inertia to each segment. Default measures are for males. Obtained from
   Paolo de Leva, 1996, Journal of Biomechanics, Vol 29, pp 1223-1230 *}
   
   {* Please not that currently there are no inertial properties for children.  Entering 1 for Adult 
   will result in no kinetics being calculated.*}

 If $Female <> 1 and $Adult <> 0
	FemurMass=0.1416
	FemPos={0,0.5905,0}
	FemRG={0.329*0.329,0.149*0.149,0.329*0.329}

	TibiaMass=0.0433
	TibiaPos={0,0.5605,0}
	TibiaRG={0.251*0.251,0.102*0.102,0.246*0.246}
	
	FootMass=0.0137
	FootPos={0.5585,0,0}
	FootRG={0.124*0.124,0.257*0.257,0.245*0.245}

	TrunkMass=0.4346
	TrunkPos={0,0.4862,0}
	TrunkRG={0.328*0.328,0.169*0.169,0.306*0.306}
	
	UTrunkMass=0.1596
	UTrunkPos={0,0.4934,0}
	UTrunkRG={0.505*0.505,0.465*0.465,0.320*0.320}
	
	HeadMass=0.0694
	HeadPos={0,0.4998,0}
	HeadRG={0.303*0.303,0.261*0.261,0.315*0.315}
	
	WTrunkMass=(0.0694+0.4346+(2*0.0271)+(2*0.0162)+(2*0.0061))
 EndIf
 If $Female <> 0 and $Adult <> 0
	FemurMass=0.1478
	FemPos={0,0.6388,0}
	FemRG={0.369*0.369,0.162*0.162,0.364*0.364}

	TibiaMass=0.0481
	TibiaPos={0,0.5648,0}
	TibiaRG={0.267*0.267,0.092*0.092,0.263*0.263}
	
	FootMass=0.0129
	FootPos={0.5986,0,0}
	FootRG={0.139*0.139,0.299*0.299,0.279*0.279}

	TrunkMass=0.4257
	TrunkPos={0,0.5036,0}
	TrunkRG={0.307*0.307,0.147*0.147,0.292*0.292}
	
	UTrunkMass=0.1545
	UTrunkPos={0,0.4950,0}
	UTrunkRG={0.466*0.466,0.449*0.449,0.314*0.314}
	
	HeadMass=0.0668
	HeadPos={0,0.5159,0}
	HeadRG={0.271*0.271,0.261*0.261,0.295*0.295}
	
	WTrunkMass = (0.0668+0.4257+(2*0.0255)+(2*0.0138)+(2*0.0056))
 EndIf

{*======================================================================================*}
{* 				END Initialisations					*}
{*======================================================================================*}

{* 			   Start Dynamic Segmental Modeling 		    		*}
{* =====================================================================================*}

{* 					Head 				    		*}
{* =====================================================================================*}
{* use this replace macro if there are missing head markers, which cannot be interpolated *}
{* SUBSTITUTE4(LFHD, LBHD, RFHD, RBHD)*}

	Height = $Height
	HeadOrigin = (LFHD+RFHD)/2
	LeftSideHead = (LFHD+LBHD)/2
	RightSideHead = (RFHD+RBHD)/2
	MidBackHead = (LBHD+RBHD)/2
	HeadMid = (LFHD+RFHD+LBHD+RBHD)/4
	HeadSize = DIST(HeadOrigin,MidBackHead)

	Head = [HeadMid,LeftSideHead-RightSideHead,HeadOrigin-MidBackHead,zyx]
	DrawSegment(Head)

{*==============================================================*}
{*			    Pelvis				*}
{*==============================================================*}
{*Pelvis created prior to thorax as pelvis and HJC's required for torso definition*}
{* use this replace macro if there are missing thorax or torso markers, which cannot be interpolated *}
{* USE WISELY!!*}
{*SUBSTITUTE4(LASI,RASI,LPSI,RPSI)*}

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
	{* Uses HJC location of Orthotrak (Shea et al.1997 Gait and Posture 5,157) *}
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

	If $OptimalHip==2
	{* Uses HJC location previously defined using the SCoRE operation within Nexus *}
		
		InterASISDist=$InterASISdist
			If $InterASISdist == 0
				InterASISDist=DIST(LASI,RASI)
			EndIf
		aa = InterASISDist/2
		
		RHJC = $Pelvis_RThigh_score*Pelvis
		LHJC = $Pelvis_LThigh_score*Pelvis

		OUTPUT(LHJC,RHJC)

	EndIf
	
	
	DrawSegment(Pelvis)

{*=============Bridging Protocol===================

	RPelvPerp= PERP(PelOrigin,LLFC,LACR)
	LPelvPerp= PERP(PelOrigin,RLFC,LACR)
	OUTPUT(RPelvPerp,LPelvPerp)

	RPelvPerpDist= DIST(PelOrigin,RPelvPerp)
	LPelvPerpDist= DIST(PelOrigin,LPelvPerp)
	OUTPUT(RPelvPerpDist,LPelvPerpDist)

	PelvDropDist= DIST(PelOrigin,OAnatomy)*}

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

{*Creates dummy thorax segment for when a pointer has been used for C7, T10 or Sternum*}

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
	
{* Output pointer defined anatomical points relative to dummy segment. Reads the parameter created during the pointer trials *}

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
	
{*Create Thorax Segment. Allowance for virtual C7, T10 and sternum markers added June 2016*}

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
	
{* Create Virtual C7, MidHip Joint Centre and Sternum points relative to respective Segments *}
{*
	CreateRelPoint(C7,Thorax,C7relThorax)
	OUTPUT(C7relThorax)

	CreateRelPoint(MidHJC,TorsoAndLowerTorso,MidHJCrelTorsoAndLowerTorso)
	
	CreateRelPoint(STRN,TorsoAndLowerTorso,STRNrelTorsoAndLowerTorso)

	{* Create Omphalion*}
	{* Trunk percentages were created from data in Paolo de Leva *}

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

	{* Create Middle and Lower Torso Segments *}

	TorsoOMPH = [TorsoOrigin,TorsoOrigin-OMPH, STRN-T10, yzx]
	DrawSegment(TorsoOMPH)

	LowerTorso = [LowerTorsoOrigin,OMPH-LowerTorsoOrigin, LASI-RASI, yxz]
	DrawSegment(LowerTorso)
*}

{* =====================================================================================*}
{* Align the positive direction of the global X-axis with the 
average direction of the sacrum to MidPelvis vector.  Resultant segment is call Aanatomy*}
{* =====================================================================================*}

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

{*======================================================================================*}
{*			    Femur							*}
{*======================================================================================*}
{* use this replace macro if you are missing thigh markers, which cannot be interpolated *}

{*
SUBSTITUTE4(LTH1,LTH2,LTH3,LHJC)
SUBSTITUTE4(RTH1,RTH2,RTH3,RHJC)
*}

{* 
Create dummy left and right femur. Fourth marker added June 2016 by Dan Cottam
If markers 1-3 are missing their positions are recreated by using the fourth marker (as long as all four markers were present in the static). This means the dummy segment will be constructed as normal.
*}

{*Dummy Left Femur*}

	If ExistAtAll (LTH1,LTH2,LTH3)
		LeftThighOrigin=(LTH1+LTH2+LTH3)/3
	ELSIF ExistAtAll (LTH2,LTH3,LTH4,$LTH1RelTech)
		LeftThighOrigin2=(LTH2+LTH3+LTH4)/3
		DummyLeftFemur2 = [LeftThighOrigin2,LeftThighOrigin2-LTH3,LTH2-LTH4,yzx]
		LTH1=$LTH1RelTech*DummyLeftFemur2
		OUTPUT(LTH1)
		LeftThighOrigin=(LTH1+LTH2+LTH3)/3
	ELSIF ExistAtAll (LTH1,LTH3,LTH4,$LTH2RelTech)
		LeftThighOrigin3=(LTH1+LTH3+LTH4)/3
		DummyLeftFemur3 = [LeftThighOrigin3,LTH1-LTH3,LeftThighOrigin3-LTH4,yzx]
		LTH2=$LTH2RelTech*DummyLeftFemur3
		OUTPUT(LTH2)
		LeftThighOrigin=(LTH1+LTH2+LTH3)/3
	ELSE ExistAtAll (LTH1,LTH2,LTH4,$LTH3RelTech)
		LeftThighOrigin4=(LTH1+LTH2+LTH4)/3
		DummyLeftFemur4 = [LeftThighOrigin4,LTH1-LeftThighOrigin4,LTH2-LTH4,yzx]
		LTH3=$LTH3RelTech*DummyLeftFemur4
		OUTPUT(LTH3)
		LeftThighOrigin=(LTH1+LTH2+LTH3)/3
	EndIf

	DummyLeftFemur = [LeftThighOrigin,LTH1-LTH3,LTH2-LeftThighOrigin,yzx] 
	OUTPUT(LeftThighOrigin)

{*Dummy Right Femur*}
	
	If ExistAtAll (RTH1,RTH2,RTH3)
		RightThighOrigin=(RTH1+RTH2+RTH3)/3
	ELSIF ExistAtAll (RTH2,RTH3,RTH4,$RTH1RelTech)
		RightThighOrigin2=(RTH2+RTH3+RTH4)/3
		DummyRightFemur2 = [RightThighOrigin2,RightThighOrigin2-RTH3,RTH2-RTH4,yzx]
		RTH1=$RTH1RelTech*DummyRightFemur2
		OUTPUT(RTH1)
		RightThighOrigin=(RTH1+RTH2+RTH3)/3
	ELSIF ExistAtAll (RTH1,RTH3,RTH4,$RTH2RelTech)
		RightThighOrigin3=(RTH1+RTH3+RTH4)/3
		DummyRightFemur3 = [RightThighOrigin3,RTH1-RTH3,RightThighOrigin3-RTH4,yzx]
		RTH2=$RTH2RelTech*DummyRightFemur3
		OUTPUT(RTH2)
		RightThighOrigin=(RTH1+RTH2+RTH3)/3
	ELSE ExistAtAll (RTH1,RTH2,RTH4,$RTH3RelTech)
		RightThighOrigin4=(RTH1+RTH2+RTH4)/3
		DummyRightFemur4 = [RightThighOrigin4,RTH1-RightThighOrigin4,RTH2-RTH4,yzx]
		RTH3=$RTH3RelTech*DummyRightFemur4
		OUTPUT(RTH3)
		RightThighOrigin=(RTH1+RTH2+RTH3)/3
	EndIf

	DummyRightFemur = [RightThighOrigin,RTH1-RTH3,RTH2-RightThighOrigin,yzx] 
	OUTPUT(RightThighOrigin)


{* Output anatomical points relative to dummy segment *}

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

{* Output pointer defined anatomical points relative to dummy segment *}

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

{*Output functionally defined knee joint axes*}

	If $HelicalKnee == 1
		LKHel1=$LKHA1*DummyLeftFemur
		LKHel2=$LKHA2*DummyLeftFemur  
		RKHel1=$RKHA1*DummyRightFemur
		RKHel2=$RKHA2*DummyRightFemur
		OUTPUT(LKHel1,LKHel2,RKHel1,RKHel2)
	EndIf
	
	If $HelicalKnee == 2
	{* Uses KJC/axis of rotation markers previously defined using the SCoRE/SARA operation within Nexus *}

		LKHel1=$LKHA1*DummyLeftFemur
		LKHel2=$LKHA2*DummyLeftFemur  
		RKHel1=$RKHA1*DummyRightFemur
		RKHel2=$RKHA2*DummyRightFemur
		OUTPUT(LKHel1,LKHel2,RKHel1,RKHel2)
		
	EndIf

{* Define Knee Joint Centres*}

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

{*Redefine KJC using functionally defined axes*}

	If $HelicalKnee > 0
		LKJC=PERP(LKJC,LKHel1,LKHel2)
		RKJC=PERP(RKJC,RKHel1,RKHel2) 
		OUTPUT(LKJC,RKJC)	
			
	{* check to make sure the helical axes vectors are in the right direction *}

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

{*==============================================================*}
{*			    Tibia				*}
{*==============================================================*}

{* only use this replace macro if desperate!  and you have missing tibia markers!*}
{*
SUBSTITUTE4(LTB1,LTB2,LTB3,LKJC)
SUBSTITUTE4(RTB1,RTB2,RTB3,RKJC)
*}

{*Create dummy tibia segments. Fourth marker added June 2016.*}

{*Left Dummy Tibia*}

	If ExistAtAll(LTB1,LTB2,LTB3)
		LeftTibOrigin=(LTB1+LTB2+LTB3)/3
	ELSIF ExistAtAll (LTB2,LTB3,LTB4,$LTB1RelTech)
		LeftTibOrigin2=(LTB2+LTB3+LTB4)/3
		DummyLeftTibia2 = [LeftTibOrigin2,LeftTibOrigin2-LTB3,LTB2-LTB4,yzx]
		LTB1=$LTB1RelTech*DummyLeftTibia2
		OUTPUT(LTB1)
	ELSIF ExistAtAll (LTB1,LTB3,LTB4,$LTB2RelTech)
		LeftTibOrigin3=(LTB1+LTB3+LTB4)/3
		DummyLeftTibia3 = [LeftTibOrigin3,LTB1-LTB3,LeftTibOrigin3-LTB4,yzx]
		LTB2=$LTB2RelTech*DummyLeftTibia3
		OUTPUT(LTB2)
	ELSIF ExistAtAll (LTB1,LTB2,LTB4,$LTB2RelTech)
		LeftTibOrigin4=(LTB2+LTB3+LTB4)/3
		DummyLeftTibia4 = [LeftTibOrigin4,LTB1-LeftTibOrigin4,LTB2-LTB4,yzx]
		LTB3=$LTB3RelTech*DummyLeftTibia4
		OUTPUT(LTB3)
	EndIf
	
	DummyLeftTibia = [LeftTibOrigin,LTB1-LTB3,LTB2-LeftTibOrigin,yzx]
	OUTPUT (LeftTibOrigin)
	
{*Right Dummy Tibia*}
	
	If ExistAtAll (RTB1,RTB2,RTB3)
		RightTibOrigin=(RTB1+RTB2+RTB3)/3
	ELSIF ExistAtAll (RTB2,RTB3,RTB4,$RTB1RelTech)
		RightTibOrigin2=(RTB2+RTB3+RTB4)/3
		DummyRightTibia2 = [RightTibOrigin2,RightTibOrigin2-RTB3,RTB2-RTB4,yzx]
		RTB1=$RTB1RelTech*DummyRightTibia2
		OUTPUT(RTB1)
	ELSIF ExistAtAll (RTB1,RTB3,RTB4,$RTB2RelTech)
		RightTibOrigin3=(RTB1+RTB3+RTB4)/3
		DummyRightTibia3 = [RightTibOrigin3,RTB1-RTB3,RightTibOrigin3-RTB4,yzx]
		RTB2=$RTB2RelTech*DummyRightTibia3
		OUTPUT(RTB2)
	ELSIF ExistAtAll (RTB1,RTB2,RTB4,$RTB3RelTech)
		RightTibOrigin4=(RTB1+RTB2+RTB4)/3
		DummyRightTibia4 = [RightTibOrigin4,RTB1-RightTibOrigin4,RTB2-RTB4,yzx]
		RTB3=$RTB3RelTech*DummyRightTibia4
		OUTPUT(RTB3)
	EndIf

	DummyRightTibia = [RightTibOrigin,RTB1-RTB3,RTB2-RightTibOrigin,yzx]
	OUTPUT(RightTibOrigin)

{*Reconstruct Ankle Malleoli*}

	RMMAL=$RMMALAnatRelTech*DummyRightTibia
	RLMAL=$RLMALAnatRelTech*DummyRightTibia
	LMMAL=$LMMALAnatRelTech*DummyLeftTibia
	LLMAL=$LLMALAnatRelTech*DummyLeftTibia
	OUTPUT(LMMAL,LLMAL,RMMAL,RLMAL)

{* Find ankle joint centres*}

	LAJC = ((LMMAL+LLMAL)/2)
	RAJC = ((RLMAL+RMMAL)/2)
	
	OUTPUT(LAJC,RAJC)

{*Define left and right tibia segments*}	
	
	LeftTibia = [LAJC,LKJC-LAJC,LLMAL-LMMAL,yxz]
	RightTibia = [RAJC,RKJC-RAJC,RMMAL-RLMAL,yxz]
 
{*Apply tibial rotation offset. This offset should be defined as the difference between the knee and ankle FE axes during the static trial. This is done within UWALBStatic.mod  *}

	
	LeftTibia=ROT(LeftTibia,2(LeftTibia),$LTibRotOffset)
	RightTibia=ROT(RightTibia,-2(RightTibia),$RTibRotOffset)
	
	DrawSegment(LeftTibia)
	DrawSegment(RightTibia)

{*==============================================================*}
{*			    Foot				*}
{*==============================================================*}


{* Each foot has three real (L/RCAL,L/RMT1,L/RMT5)markers amd one virtual(LFTML) marker 
   Note that the foot markers are not necessarily placed on anatomical landmarks. This
   is basically a dummy segment and the calibration rig is used to create virtual markers
   that properly define the foot segment. *}

{* only use this replace macro if you have missing foot markers*}

{*
SUBSTITUTE4(LMT1,LMT5,LCAL,LAJC)
SUBSTITUTE4(RMT1,RMT5,RCAL,RAJC)
*}
{* Recreate position*}
{*
FootTechRMT1Origin = (RLMAL+RMMAL+RMT5)/3
FootTechRMT1 = [FootTechRMT1Origin, RLMAL-RMMAL, RMT5-FootTechRMT1Origin, zyx]

RMT1 = $RMT1a*FootTechRMT1
OUTPUT(RMT1)
*}

LFTML=(LMT5+LMT1)/2
RFTML=(RMT5+RMT1)/2
LeftFootOrigin=LFTML
RightFootOrigin=RFTML
OUTPUT(LFTML,RFTML) 
 
DummyLeftFoot = [LCAL,LeftFootOrigin-LCAL,LMT1-LMT5,xyz]
DummyRightFoot = [RCAL,RightFootOrigin-RCAL,RMT5-RMT1,xyz]

{* define more virtual points relative to dummy segment. These points are determined
   from the calibration rig and their relative reference to the dummy segment is 
   calculated in UWAStaticLB_2010.mod *}

LFootMidline=$LFMIDvirtual*DummyLeftFoot
RFootMidline=$RFMIDvirtual*DummyRightFoot
LFootSide=$LFSIDEvirtual*DummyLeftFoot
RFootSide=$RFSIDEvirtual*DummyRightFoot
OUTPUT(LFootMidline,RFootMidline,LFootSide,RFootSide)

LeftFoot = [LCAL,LFootMidline-LCAL,LFootSide-LCAL,xyz]
RightFoot = [RCAL,RFootMidline-RCAL,RFootSide-RCAL,xyz]

DrawSegment(LeftFoot)
DrawSegment(RightFoot)



{*======================================================================================*}
{* 					Ball              				*}
{*======================================================================================*}

	BallOrigin = (BALL2+BALL4)/2
	Ball = [BallOrigin,BALL2-BALL4,BallOrigin-BALL1,zyx]
	OUTPUT(BallOrigin)
	DrawSegment(Ball)

	LFootBallAngle = -<LeftFoot, Ball, zyx>
	RFootBallAngle = -<RightFoot, Ball, zyx>
	BallAngleGlob = -<Anatomy, Ball, zyx>
	OUTPUT(LFootBallAngle, RFootBallAngle, BallAngleGlob)

{* Start of the Drawing Section. This is all for looks! *}
{* =====================================================================================*}

{* Start of the Drawing Section	*}
{* =====================================================================================*}

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
	
	LFoot = [LFootMidline,LFootMidline-LCAL,LFootSide-LCAL,xzy]
			 
	LFOO1 = $LFootScale*{65,-25,-25}*LFoot
	LFOO2 = $LFootScale*{45,20,-25}*LFoot
	LFOO3 = $LFootScale*{-25,10,-25}*LFoot
	LFOO4 = $LFootScale*{-25,-10,-25}*LFoot
	LFOO5 = $LFootScale*{-20,0,10}*LFoot
	LFOO6 = $LFootScale*{5,0,10}*LFoot

	OUTPUT(LFOO1,LFOO2,LFOO3,LFOO4,LFOO5,LFOO6)

{*Draw RFoot, using 6 vertices, scaled to bone length*}	
	
	RFoot = [RFootMidline,RFootMidline-RCAL,RFootSide-RCAL,xzy]

	RFOO1 = $RFootScale*{65,25,-25}*RFoot
	RFOO2 = $RFootScale*{45,-20,-25}*RFoot
	RFOO3 = $RFootScale*{-25,-10,-25}*RFoot
	RFOO4 = $RFootScale*{-25,10,-25}*RFoot
	RFOO5 = $RFootScale*{-20,0,10}*RFoot
	RFOO6 = $RFootScale*{5,0,10}*RFoot

	OUTPUT(RFOO1,RFOO2,RFOO3,RFOO4,RFOO5,RFOO6)


{* 				END Static Segmental Modeling 				*}
{* =====================================================================================*}

{*Angle Calcs*}
{*==============================================================*}

{*Relative Angles*}

	LHipAngle = -<Pelvis,LeftFemur,zxy>
	RHipAngle = -<Pelvis,RightFemur,zxy>

	LKneeAngle = -<LeftFemur,LeftTibia,zxy>
	RKneeAngle = -<RightFemur,RightTibia,zxy>

	LAnkleAngle = -<LeftTibia,LeftFoot,zxy>
	RAnkleAngle = -<RightTibia,RightFoot,zxy>

	RFootProgression = -<Pelvis,RightFoot,zxy>
	LFootProgression = -<Pelvis,LeftFoot,zxy>
	
	ThortoPelAngle = -<Pelvis,Thorax,zxy>
	TorsotoPelAngle = -<Pelvis,Torso,zxy>
	
{*Global Angles*}

	TrunkAngleGlob = <Anatomy, Trunk, zxy>
	TrunkAngleBaker= <Anatomy, Trunk, yxz>

	ThoraxAngle 	= -<Anatomy, Thorax, zxy>
	ThoraxAngleBaker = -<Anatomy, Thorax, yxz>
	
	TorsoAngle 	= -<Anatomy, Torso, zxy>
	TorsoAngleBaker = -<Anatomy, Torso, yxz>
	
	PelvisAngle = -<Anatomy, Pelvis, zxy>
	PelvisAngleBaker = -<Anatomy, Pelvis, yxz>

	LFemurAngleGlob = -<Anatomy,LeftFemur,zxy> 
	RFemurAngleGlob = -<Anatomy,RightFemur,zxy> 

	LTibiaAngleGlob = -<Anatomy,LeftTibia,zxy> 
	RTibiaAngleGlob = -<Anatomy,RightTibia,zxy> 

	RFootAngleGlob = -<Anatomy,RightFoot,zxy>
	LFootAngleGlob = -<Anatomy,LeftFoot,zxy>

{*Linear Velocity Calcs*}
{* ==============================================================*}

	Linvelacc(RAJC)
	Linvelacc(LAJC)
	Linvelacc(RKJC)
	Linvelacc(LKJC)
	Linvelacc(RHJC)
	Linvelacc(LHJC)
	Linvelacc(MidPelvis)
	Linvelacc(ThoraxOrigin)
	Linvelacc(TorsoOrigin)
	Linvelacc(C7)
	Linvelacc(BallOrigin)


{*Angular Velocity Calcs*}
{* ==============================================================*}

	ANGVELACC(RightFemur,Pelvis,RHip)
	ANGVELACC(LeftFemur,Pelvis,LHip)
	ANGVELACC(RightTibia,RightFemur,RKnee)
	ANGVELACC(LeftTibia,LeftFemur,LKnee)
	ANGVELACC(RightFoot,RightTibia,RAnkle)
	ANGVELACC(LeftFoot,LeftTibia,LAnkle)

	ANGVELACC(Thorax,Anatomy,ThorGlob)
	ANGVELACC(Pelvis,Anatomy,PelGlob)
	ANGVELACC(Torso,Anatomy,TorsoGlob)
	{*ANGVELACC(LowerTorso,Anatomy,LTorsoGlob)*}

	ANGVELACC(Thorax,Pelvis,ThorPel)
	ANGVELACC(Torso,Pelvis,TorsoPel)
	{*ANGVELACC(LowerTorso,Pelvis,LTorsoPel)*}

	OUTPUT(ThorPelAngVelDeg,TorsoPelAngVelDeg,ThorPelAngAccDeg,TorsoPelAngAccDeg)


{*KINETICS*}
{* =====================================================================================*}
{* Limit moving average filter *}
REACTIONFILTER = 0
POWERFILTER = 0


{*      Anthropometry    *}
{* --------------------- *}

{* =====================================================================================*}
{* General length definitions must be calculated first before Kinetic data can be obtained*}

{*Average functions are not permitted in If statements. Therefore C7TrunkLength and VC7TrunkLength need to be calculated separately before being renamed as TrunkLength*}

	VC7TrunkLength=Average(DIST(VC7,MidHJC))
	C7TrunkLength=Average(DIST(C7,MidHJC))

	If ExistAtAll (VC7)
		TrunkLength=VC7TrunkLength
	ELSE TrunkLength=C7TrunkLength
	EndIf

	TrunkLength2 = TrunkLength*TrunkLength
	LFemurLength = Average(DIST(LHJC,LKJC))
	LFemurLength2 = LFemurLength*LFemurLength
	RFemurLength = Average(DIST(RHJC,RKJC))
	RFemurLength2 = RFemurLength*RFemurLength
	LTibiaLength = Average(DIST(LKJC,LAJC))
	LTibiaLength2 = LTibiaLength*LTibiaLength
	RTibiaLength = Average(DIST(RKJC,RAJC))
	RTibiaLength2 = RTibiaLength*RTibiaLength
	LFootLength = $LFootLength
	LFootLength2 = LFootLength*LFootLength
	RFootLength = $RFootLength
	RFootLength2 = RFootLength*RFootLength
	LJMCOMLength = Average(TrunkLength+LFemurLength+RFemurLength+LTibiaLength+RTibiaLength+LFootLength+RFootLength)
	LJMCOMLength2 = LJMCOMLength*LJMCOMLength

{* This following code moves the origins of the segments to their centre of masses 
   Note that this means that in the inertial definitions for COM position is now (0,0,0) *}

{* Define COM locations *}
	LeftFemurCOMGlobal  = (LFemurLength*FemPos)*LeftFemur
	LeftTibiaCOMGlobal  = (LTibiaLength*TibiaPos)*LeftTibia
	LeftFootCOMGlobal   = (LFootLength*FootPos)*LeftFoot
	RightFemurCOMGlobal = (RFemurLength*FemPos)*RightFemur
	RightTibiaCOMGlobal = (RTibiaLength*TibiaPos)*RightTibia
	RightFootCOMGlobal  = (RFootLength*FootPos)*RightFoot
	TrunkCOMGlobal 		= (TrunkLength*TrunkPos)*Trunk
	OUTPUT(LeftFemurCOMGlobal,LeftTibiaCOMGlobal,LeftFootCOMGlobal,RightFemurCOMGlobal,RightTibiaCOMGlobal,RightFootCOMGlobal,TrunkCOMGlobal)
	
	Linvelacc(TrunkCOMGlobal)

{* TrunkCoMtoFoot *}

	TrunkCOMtoRFoot = (TrunkCOMGlobal-RAJC)
	TrunkCOMtoLFoot = (TrunkCOMGlobal-LAJC)
	TrunkCOMtoRFootConv = TrunkCOMtoRFoot(2)*-1
	OUTPUT(TrunkCOMtoRFoot,TrunkCOMtoLFoot,TrunkCOMtoRFootConv)
	PARAM(TrunkCOMtoRFoot,TrunkCOMtoLFoot,TrunkCOMtoRFootConv)

{* LiverPool CoM *}

	LJMCOMGlobal = ((TrunkCOMGlobal*TrunkMass)+(LeftFemurCOMGlobal*FemurMass)+(LeftTibiaCOMGlobal*TibiaMass)+(LeftFootCOMGlobal*FootMass)+(RightFemurCOMGlobal*FemurMass)+(RightTibiaCOMGlobal*TibiaMass)+(RightFootCOMGlobal*FootMass))/(TrunkMass+FemurMass+FemurMass+TibiaMass+TibiaMass+FootMass+FootMass)

	FBCOMGlobal = ((TrunkCOMGlobal*WTrunkMass)+(LeftFemurCOMGlobal*FemurMass)+(LeftTibiaCOMGlobal*TibiaMass)+(LeftFootCOMGlobal*FootMass)+(RightFemurCOMGlobal*FemurMass)+(RightTibiaCOMGlobal*TibiaMass)+(RightFootCOMGlobal*FootMass))/(WTrunkMass+FemurMass+FemurMass+TibiaMass+TibiaMass+FootMass+FootMass)
	
	LJMCOMGlobaltoRFoot = (LJMCOMGlobal-RAJC)
	LJMCOMGlobaltoLFoot = (LJMCOMGlobal-LAJC)
	LJMCOMGlobaltoRFootConv = LJMCOMGlobaltoRFoot(2)*-1
	FBCOMGlobaltoRFoot = (LJMCOMGlobal-RAJC)
	FBCOMGlobaltoLFoot = (LJMCOMGlobal-LAJC)
	FBCOMGlobaltoRFootConv = LJMCOMGlobaltoRFoot(2)*-1
	OUTPUT(FBCOMGlobal,LJMCOMGlobal,LJMCOMGlobaltoRFoot,LJMCOMGlobaltoLFoot,LJMCOMGlobaltoRFootConv,FBCOMGlobaltoRFoot,FBCOMGlobaltoLFoot,FBCOMGlobaltoRFootConv)
	PARAM(FBCOMGlobal,LJMCOMGlobal,LJMCOMGlobaltoRFoot,LJMCOMGlobaltoLFoot,LJMCOMGlobaltoRFootConv,FBCOMGlobaltoRFoot,FBCOMGlobaltoLFoot,FBCOMGlobaltoRFootConv)

	Linvelacc(LJMCOMGlobal)
	Linvelacc(FBCOMGlobal)

{* Translate segment origins from distal end of segments to segment COMs *}

	Trunk = TrunkCOMGlobal + ATTITUDE(Trunk)
	LeftFemur  = LeftFemurCOMGlobal  + ATTITUDE(LeftFemur)
	LeftTibia  = LeftTibiaCOMGlobal  + ATTITUDE(LeftTibia) 
	RightFemur = RightFemurCOMGlobal + ATTITUDE(RightFemur) 
	RightTibia = RightTibiaCOMGlobal + ATTITUDE(RightTibia) 
	RightFoot  = RightFootCOMGlobal  + ATTITUDE(RightFoot) 
	LeftFoot   = LeftFootCOMGlobal   + ATTITUDE(LeftFoot) 

	DrawSegment(Trunk)
	DrawSegment(LeftTibia) 
	DrawSegment(RightTibia) 
	DrawSegment(LeftFemur) 
	DrawSegment(RightFemur) 
	DrawSegment(LeftFoot)
	DrawSegment(RightFoot)

{* Definition of segment inertial parameters *}
	
	{*
	Head = [Head, $BodyMass*HeadMass, {0,0,0},HeadLength2*HeadRG*HeadMass*$BodyMass]
	*}

	Trunk = [Trunk, $BodyMass*TrunkMass, {0,0,0},TrunkLength2*TrunkRG*TrunkMass*$BodyMass]
	Pelvis = [Pelvis, $BodyMass*0.142, {0,0,0}, {0,0,0}]
	LeftFemur=[LeftFemur, Pelvis, LHJC, FemurMass*$BodyMass, {0,0,0},LFemurLength2*FemRG*FemurMass*$BodyMass]
	RightFemur=[RightFemur, Pelvis, RHJC, FemurMass*$BodyMass, {0,0,0},RFemurLength2*FemRG*FemurMass*$BodyMass]
	RightTibia=[RightTibia, RightFemur, RKJC, $BodyMass*TibiaMass, {0,0,0},RTibiaLength2*TibiaRG*$BodyMass*TibiaMass]
	LeftTibia=[LeftTibia, LeftFemur, LKJC, $BodyMass*TibiaMass, {0,0,0},LTibiaLength2*TibiaRG*$BodyMass*TibiaMass]
	LeftFoot=[LeftFoot, LeftTibia, LAJC, FootMass*$BodyMass, {0,0,0},LFootLength2*FootRG*FootMass*$BodyMass]
	RightFoot=[RightFoot, RightTibia, RAJC, FootMass*$BodyMass, {0,0,0},RFootLength2*FootRG*FootMass*$BodyMass]

{* Inverse Dynamic Calcs *}
{* --------------------- *}

	LHipReaction   = REACTION(LeftFemur,LHJC/LeftFemur)
	RHipReaction   = REACTION(RightFemur,RHJC/RightFemur)
	LKneeReaction  = REACTION(LeftTibia,LKJC/LeftTibia)
	RKneeReaction  = REACTION(RightTibia,RKJC/RightTibia)
	LAnkleReaction = REACTION(LeftFoot,LAJC/LeftFoot)
	RAnkleReaction = REACTION(RightFoot,RAJC/RightFoot)

	LHipForce  = LHipReaction(1)
	RHipForce  = RHipReaction(1)
	LKneeForce = LKneeReaction(1)
	RKneeForce = RKneeReaction(1)
	LAnkleForce= LAnkleReaction(1)
	RAnkleForce= RAnkleReaction(1)

{* Convert moments from N.mm to N.m *}

	LHipMoment   = LHipReaction(2) /1000
	RHipMoment   = RHipReaction(2) /1000
	LKneeMoment  = LKneeReaction(2)/1000
	RKneeMoment  = RKneeReaction(2)/1000
	LAnkleMoment = LAnkleReaction(2)/1000
	RAnkleMoment = RAnkleReaction(2)/1000

	LHipPosition  = LHipReaction(3)/1000
	RHipPosition  = RHipReaction(3)/1000
	LKneePosition = LKneeReaction(3)/1000
	RKneePosition = RKneeReaction(3)/1000
	LAnklePosition= LAnkleReaction(3)/1000
	RAnklePosition= RAnkleReaction(3)/1000

{* Calculate Joint Powers using Vicon function and convert from Milliwatts to Watts)  *}
{* ---------------------------------------------------------------------------------- *}

	LPowerHip   = POWER (Pelvis,LeftFemur)/1000
	LPowerKnee  = POWER (LeftFemur, LeftTibia)/1000
	LPowerAnkle = POWER (LeftTibia, LeftFoot)/1000
	RPowerHip   = POWER (Pelvis, RightFemur)/1000
	RPowerKnee  = POWER (RightFemur, RightTibia)/1000
	RPowerAnkle = POWER (RightTibia, RightFoot)/1000

{* Normalise power output by mass (W/Kg) *}

	LPowerHipNorm   = LPowerHip /$BodyMass
	LPowerKneeNorm  = LPowerKnee /$BodyMass
	LPowerAnkleNorm = LPowerAnkle/$BodyMass
	RPowerHipNorm   = RPowerHip /$BodyMass
	RPowerKneeNorm  = RPowerKnee/$BodyMass
	RPowerAnkleNorm = RPowerAnkle/$BodyMass


{* Calculate joint power terms using UWA macro (W) *}
{* ----------------------------------------------- *}

	JOINTPOWER(RightFemur,Pelvis,RHip)
	JOINTPOWER(LeftFemur,Pelvis,LHip)
	JOINTPOWER(RightTibia,RightFemur,RKnee)
	JOINTPOWER(LeftTibia,LeftFemur,LKnee)
	JOINTPOWER(RightFoot,RightTibia,RAnkle)
	JOINTPOWER(LeftFoot,LeftTibia,LAnkle)

{* ---------------------------------------------- *}
{*    Putting things in right axis conventions    *}
{* ---------------------------------------------- *}

{* Angle Outputs *}

	TrunkAngleGlob = <-1(TrunkAngleGlob),2(TrunkAngleGlob),3(TrunkAngleGlob)>
	TrunkAngleBaker = <1(TrunkAngleBaker),2(TrunkAngleBaker),-3(TrunkAngleBaker)>
	
	ThoraxAngle = <-1(ThoraxAngle),2(ThoraxAngle),3(ThoraxAngle)>
	ThoraxAngleBaker = <1(ThoraxAngleBaker),2(ThoraxAngleBaker),-3(ThoraxAngleBaker)>

	TorsoAngle = <-1(TorsoAngle),2(TorsoAngle),3(TorsoAngle)>
	TorsoAngleBaker = <1(TorsoAngleBaker),2(TorsoAngleBaker),-3(TorsoAngleBaker)>
	
	ThortoPelAngle = <-1(ThortoPelAngle),2(ThortoPelAngle),3(ThortoPelAngle)>
	TorsotoPelAngle = <-1(TorsotoPelAngle),2(TorsotoPelAngle),3(TorsotoPelAngle)>
	
	PelvisAngle = <-1(PelvisAngle),2(PelvisAngle),3(PelvisAngle)>
	PelvisAngleBaker = <1(PelvisAngleBaker),2(PelvisAngleBaker),-3(PelvisAngleBaker)>

	LHipAngle = <1(LHipAngle),-2(LHipAngle),-3(LHipAngle)> 
	RHipAngle = <1(RHipAngle),2(RHipAngle),3(RHipAngle)> 

	LKneeAngle = <-1(LKneeAngle),-2(LKneeAngle),-3(LKneeAngle)> 
	RKneeAngle = <-1(RKneeAngle),2(RKneeAngle),3(RKneeAngle)>

	LAnkleAngle = <1(LAnkleAngle),-2(LAnkleAngle),-3(LAnkleAngle)>
	RAnkleAngle = <1(RAnkleAngle),2(RAnkleAngle),3(RAnkleAngle)>

	RFootProgression = <1(RFootProgression),2(RFootProgression),3(RFootProgression)> 
	LFootProgression  = <1(LFootProgression),-2(LFootProgression),-3(LFootProgression)> 

	RTibiaAngleGlob = <1(RTibiaAngleGlob),2(RTibiaAngleGlob),3(RTibiaAngleGlob)> 
	LTibiaAngleGlob = <1(LTibiaAngleGlob),-2(LTibiaAngleGlob),-3(LTibiaAngleGlob)>

	RFemurAngleGlob = <1(RFemurAngleGlob),2(RFemurAngleGlob),3(RFemurAngleGlob)> 
	LFemurAngleGlob = <1(LFemurAngleGlob),-2(LFemurAngleGlob),-3(LFemurAngleGlob)>

	RFootAngleGlob = <1(RFootAngleGlob),2(RFootAngleGlob),3(RFootAngleGlob)> 
	LFootAngleGlob = <1(LFootAngleGlob),-2(LFootAngleGlob),-3(LFootAngleGlob)>

	OUTPUT(RFootProgression,LFootProgression,TrunkAngleGlob,TrunkAngleBaker,ThoraxAngle,ThoraxAngleBaker,TorsoAngle,TorsoAngleBaker,PelvisAngle,PelvisAngleBaker)
	OUTPUT(LHipAngle,RHipAngle,LKneeAngle,RKneeAngle)
	OUTPUT(LAnkleAngle,RAnkleAngle,RFootAngleGlob,LFootAngleGlob,RTibiaAngleGlob,LTibiaAngleGlob,RFemurAngleGlob,LFemurAngleGlob,ThortoPelAngle,TorsotoPelAngle)

{*===============================================================================*}
{*         FKM Joint Centres relative to anatomical coordinate systems            *}
{*===============================================================================*}

	RHJCRelPelvis = RHJC/Pelvis
	LHJCRelPelvis = LHJC/Pelvis
	MidPelRelPelvis = MidPelvis/Pelvis
	RHJCRelRightFemur = RHJC/RightFemur
	LHJCRelLeftFemur = LHJC/LeftFemur
	RKJCRelRightFemur = RKJC/RightFemur
	LKJCRelLeftFemur = LKJC/LeftFemur
	RKJCRelRightTibia = RKJC/RightTibia
	LKJCRelLeftTibia = LKJC/LeftTibia
	RAJCRelRightTibia = RAJC/RightTibia
	LAJCRelLeftTibia = LAJC/LeftTibia
	RAJCRelRightFoot = RAJC/RightFoot
	LAJCRelLeftFoot = LAJC/LeftFoot
	RFTMLRelRightFoot = RFTML/RightFoot
	LFTMLRelLeftFoot = LFTML/LeftFoot

	OUTPUT(RHJCRelPelvis,LHJCRelPelvis,MidPelRelPelvis)
	OUTPUT(RHJCRelRightFemur,LHJCRelLeftFemur,RKJCRelRightFemur,LKJCRelLeftFemur)
	OUTPUT(RKJCRelRightTibia,LKJCRelLeftTibia,RAJCRelRightTibia,LAJCRelLeftTibia)
	OUTPUT(RAJCRelRightFoot,LAJCRelLeftFoot,RFTMLRelRightFoot,LFTMLRelLeftFoot)

{* Angular velocities/accelerations *}

	PelGlobAngVel	= {-3(PelGlobAngVel),1(PelGlobAngVel),2(PelGlobAngVel)}
	PelGlobAngAcc	={-3(PelGlobAngAcc),1(PelGlobAngAcc),2(PelGlobAngAcc)}
	PelGlobAngVelDeg={-3(PelGlobAngVelDeg),1(PelGlobAngVelDeg),2(PelGlobAngVelDeg)}
	PelGlobAngAccDeg= {-3(PelGlobAngAccDeg),1(PelGlobAngAccDeg),2(PelGlobAngAccDeg)}

	LHipAngVel 	= {3(LHipAngVel), -1(LHipAngVel), -2(LHipAngVel)}
	LHipAngVelDeg 	= {3(LHipAngVelDeg), -1(LHipAngVelDeg), -2(LHipAngVelDeg)}
	RHipAngVel 	= {3(RHipAngVel),  1(RHipAngVel),  2(RHipAngVel)}
	RHipAngVelDeg 	= {3(RHipAngVelDeg),  1(RHipAngVelDeg),  2(RHipAngVelDeg)}

	LKneeAngVel 	= {-3(LKneeAngVel), -1(LKneeAngVel), -2(LKneeAngVel)}
	LKneeAngVelDeg 	= {-3(LKneeAngVelDeg), -1(LKneeAngVelDeg), -2(LKneeAngVelDeg)}
	RKneeAngVel 	= {-3(RKneeAngVel),  1(RKneeAngVel),  2(RKneeAngVel)}
	RKneeAngVelDeg 	= {-3(RKneeAngVelDeg),  1(RKneeAngVelDeg),  2(RKneeAngVelDeg)}

	LAnkleAngVel 	= {3(LAnkleAngVel), -1(LAnkleAngVel), -2(LAnkleAngVel)}
	LAnkleAngVelDeg = {3(LAnkleAngVelDeg), -1(LAnkleAngVelDeg), -2(LAnkleAngVelDeg)}
	RAnkleAngVel 	= {3(RAnkleAngVel),  1(RAnkleAngVel),  2(RAnkleAngVel)}
	RAnkleAngVelDeg = {3(RAnkleAngVelDeg),  1(RAnkleAngVelDeg),  2(RAnkleAngVelDeg)}

	LHipAngAcc 	= {3(LHipAngAcc), -1(LHipAngAcc), -2(LHipAngAcc)}
	LHipAngAccDeg 	= {3(LHipAngAccDeg), -1(LHipAngAccDeg), -2(LHipAngAccDeg)}
	RHipAngAcc 	= {3(RHipAngAcc),  1(RHipAngAcc),  2(RHipAngAcc)}
	RHipAngAccDeg 	= {3(RHipAngAccDeg),  1(RHipAngAccDeg),  2(RHipAngAccDeg)}

	LKneeAngAcc 	= {-3(LKneeAngAcc), -1(LKneeAngAcc), -2(LKneeAngAcc)}
	LKneeAngAccDeg 	= {-3(LKneeAngAccDeg), -1(LKneeAngAccDeg), -2(LKneeAngAccDeg)}
	RKneeAngAcc 	= {-3(RKneeAngAcc),  1(RKneeAngAcc),  2(RKneeAngAcc)}
	RKneeAngAccDeg 	= {-3(RKneeAngAccDeg),  1(RKneeAngAccDeg),  2(RKneeAngAccDeg)}

	LAnkleAngAcc 	= {3(LAnkleAngAcc), -1(LAnkleAngAcc), -2(LAnkleAngAcc)}
	LAnkleAngAccDeg = {3(LAnkleAngAccDeg), -1(LAnkleAngAccDeg), -2(LAnkleAngAccDeg)}
	RAnkleAngAcc 	= {3(RAnkleAngAcc),  1(RAnkleAngAcc),  2(RAnkleAngAcc)}
	RAnkleAngAccDeg = {3(RAnkleAngAccDeg),  1(RAnkleAngAccDeg),  2(RAnkleAngAccDeg)}

	OUTPUT(PelGlobAngVel,PelGlobAngAcc,PelGlobAngVelDeg,PelGlobAngAccDeg)
	OUTPUT(LHipAngVel, RHipAngVel, LHipAngAcc, RHipAngAcc)
	OUTPUT(LKneeAngVel, RKneeAngVel, LKneeAngAcc, RKneeAngAcc)
	OUTPUT(LAnkleAngVel, RAnkleAngVel, LAnkleAngAcc, RAnkleAngAcc)
	OUTPUT(LHipAngVelDeg, RHipAngVelDeg, LHipAngAccDeg, RHipAngAccDeg)
	OUTPUT(LKneeAngVelDeg, RKneeAngVelDeg, LKneeAngAccDeg, RKneeAngAccDeg)
	OUTPUT(LAnkleAngVelDeg, RAnkleAngVelDeg, LAnkleAngAccDeg, RAnkleAngAccDeg)
	
{* 
Convert joint moments to functionally relevant signs.
Moment outputs are internal moments and have been adjusted so the first term is flexion/extension, the second abduction/adduction and the final term is internal/external rotation. 
*}

	LHipM  = {-3(LHipMoment),   1(LHipMoment),   2(LHipMoment)}
	RHipM  = {-3(RHipMoment),  -1(RHipMoment),  -2(RHipMoment)}
	LKneeM = { 3(LKneeMoment),  1(LKneeMoment),  2(LKneeMoment)}
	RKneeM = { 3(RKneeMoment), -1(RKneeMoment), -2(RKneeMoment)}
	LAnkleM = {-3(LAnkleMoment),-1(LAnkleMoment),-2(LAnkleMoment)}
	RAnkleM = {-3(RAnkleMoment), 1(RAnkleMoment), 2(RAnkleMoment)}

	LHipReaction   = |LHipForce, LHipM, LHipPosition|
	RHipReaction   = |RHipForce, RHipM, RHipPosition|
	LKneeReaction  = |LKneeForce, LKneeM, LKneePosition|
	RKneeReaction  = |RKneeForce, RKneeM, RKneePosition|
	LAnkleReaction = |LAnkleForce, LAnkleM, LAnklePosition|
	RAnkleReaction = |RAnkleForce, RAnkleM, RAnklePosition|

	LHipMoments = 2(LHipReaction)
	RHipMoments = 2(RHipReaction)
	LKneeMoments = 2(LKneeReaction)
	RKneeMoments = 2(RKneeReaction)
	LAnkleMoments = 2(LAnkleReaction)
	RAnkleMoments = 2(RAnkleReaction)
 
OUTPUT(LHipMoments,RHipMoments,LKneeMoments,RKneeMoments,LAnkleMoments,RAnkleMoments)
OUTPUT(LHipForce,RHipForce,LKneeForce,RKneeForce,LAnkleForce,RAnkleForce)
OUTPUT(LHipPosition,RHipPosition,LKneePosition,RKneePosition,LAnklePosition,RAnklePosition)

{* Output powers *}

	OUTPUT(LPowerHip,LPowerKnee,LPowerAnkle,RPowerHip,RPowerKnee,RPowerAnkle)
	OUTPUT(LPowerHipNorm,LPowerKneeNorm,LPowerAnkleNorm,RPowerHipNorm,RPowerKneeNorm,RPowerAnkleNorm)

	OUTPUT(RHipPowerTerms, LHipPowerTerms, RKneePowerTerms, LKneePowerTerms, RAnklePowerTerms, LAnklePowerTerms)
	OUTPUT(RHipPowerTermsNorm, LHipPowerTermsNorm, RKneePowerTermsNorm, LKneePowerTermsNorm, RAnklePowerTermsNorm, LAnklePowerTermsNorm)

{* =====Dynamic Hockey Stick Model ======*}

	STICKOrigin=(Stick1+Stick2+Stick3)/3
	OUTPUT(STICKOrigin)
	DummySTICK=[STICKOrigin,Stick1-Stick3,Stick2-STICKOrigin,zyx]

	{*Landmark in Dummy Segment Coord System*}

	StickTop=$StickTopRelTech*DummySTICK
	OUTPUT(StickTop)

	StickPost=$StickPostRelTech*DummySTICK
	OUTPUT(StickPost)

	StickInf=$StickInfRelTech*DummySTICK
	OUTPUT(StickInf)

	StickAnt=$StickAntRelTech*DummySTICK
	OUTPUT(StickAnt)

	{* Create Stick Segment *}
	STICKOrigin2=(StickPost+StickInf+StickAnt)/3
	OUTPUT(STICKOrigin2)
	If ExistAtAll(StickTop,StickPost,StickInf,StickAnt)
	Stick=[STICKOrigin2,StickTop-StickInf,StickPost-StickAnt,yxz]		
	EndIf

	DrawSegment(Stick)

	{*Stick Outputs*}
	{* GlobalStickAngle = angle between stick and global coord system *}
	{* FBCOMGlobaltoStick = distance from anterior or furthest aspect of stick to the FBCOM. COM code must be run prior to stick code for this to output *}
	{* StickHeight = height from most inf point of stick to the ground *}

	GlobalStickAngle = <Anatomy, Stick, zxy>
	FBCOMGlobaltoStick = (LJMCOMGlobal-StickAnt)
	StickHeight = DIST(StickInf,OAnatomy)
	StickHeight2 = (OAnatomy-StickInf)

	OUTPUT(GlobalStickAngle,FBCOMGlobaltoStick,StickHeight,StickHeight2) 

	
{*	
LowerBodyModelVer = {10,08,2016}
OUTPUT(LowerBodyModelVer)
$LowerBodyModelVer = 10082016
PARAM($LowerBodyModelVer)
*}
{* End of processing *}

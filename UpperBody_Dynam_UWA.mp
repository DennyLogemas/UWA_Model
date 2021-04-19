{*VICON BodyLanguage(tm)*}

{*===================================================================================================================================*}
{*====================================================UWA UPPER BODY DYNAMIC MP======================================================*}
{*===================================================================================================================================*}

{*
	1. Program-Name Creation-Date
	UWAUBDynamic.mp    30/June/2016
	2. Original-Author (Email-Address)
	Multiple Contributors
	3. Last-Updated-By (Email-Address)
	Dan Cottam (daniel.cottam@research.uwa.edu.au)
	4. Short-Description
	The generic UWA upper body dynamic bodybuilder model parameter file
	5. Notes
	Mp file is the same as the previous versions. Name changed to match new model. 
	6. Modification-History
	Version  Author     Date                 Change
	1.0       DC    30/June/2016         No content change
*}

$MarkerDiameter	= 0 {* Length Mandatory *}
$SamplingRate = 0 {*General Unitless Mandatory*}

{* if female, type 1, if male, type 0*}
$Female	= 0  {*General Unitless Mandatory*}

{* if Adult, type 1, if child, type 0*}
$Adult	= 1  {*General Unitless Mandatory*}

{* Use this to define the global anatomical CS based on subject progression *}
$SubjectBasedAnatomicalCS = 0 {*General Unitless Mandatory*}

{* Subject Specific Measurements including Manual Input From Foot Rig *}
$Height= 0 {*General Length Mandatory*}
$BodyMass = 0 {* General Mass Mandatory*}
$InterASISdist = 0 {* General Mass Mandatory*}

{* Sports Specific Parameters*}
{* if right handed, type 1, if left handed, type 0*}
$HandDom =  1   {* General Unitless Mandatory*}

{* If Optimal Hip, optimal SJC Helical Knee or Helical Elbow Found in Matlab, then type 1 *}
$HelicalWrist = 0 {* General Unitless Mandatory *}
$HelicalElbow = 0 {* General Unitless Mandatory *}
$GenericSJCOffset = 0 {* General Unitless Mandatory *}
$RegressionModel = 0 {* General Unitless Mandatory *}
$OptimalHip = 0 {* General Unitless Mandatory *}

{* Sport Specific Parameters *}

$BallMass = 0.156 {* General Mass Mandatory*}
$BallLength = 70 {* General Length Optional*}
$BallCarDist = 150 {* General Length Mandatory*}
$BallHand = 1 

{* Optional Inputs and Matlab Optimal Joint Inputs*}
$REnHA1 = {0,0,0} {* Right Elbow Length Optional*}
$REnHA2 = {0,0,0} {* Right Elbow Length Optional*}
$LEnHA1 = {0,0,0}  {* Left Elbow Length Optional*}
$LEnHA2 = {0,0,0} {* Left Elbow Length Optional*}
$REspHA1 = {0,0,0} {* Right Elbow Length Optional*}
$REspHA2 = {0,0,0} {* Right Elbow Length Optional*}
$LEspHA1 = {0,0,0}  {* Left Elbow Length Optional*}
$LEspHA2 = {0,0,0}  {* Left Elbow Length Optional*}

{* Parameters for Kinetics (calculated from CAR marker to WJC- would be more accurate to measure this!*}
$LHandLength = 0 {* Left Hand Length Calculated*}
$RHandLength = 0 {* Right Hand Length Calculated*}

$KineticParametersUB = 1 {* General Unitless Mandatory *}
FrameTimeLength = 0.004 

{* Subject Specific Parameter Information Output by Matlab or Static Model Processing*}
{* ----------------------------------------------------------------------------------*}


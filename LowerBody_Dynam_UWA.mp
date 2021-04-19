{*VICON BodyLanguage(tm)*}

{*===================================================================================================================================*}
{*====================================================UWA LOWER BODY DYNAMIC MP======================================================*}
{*===================================================================================================================================*}

{*
	1. Program-Name Creation-Date
	UWALBDynamic.mp    30/June/2016
	2. Original-Author (Email-Address)
	Multiple Contributors
	3. Last-Updated-By (Email-Address)
	Dan Cottam (daniel.cottam@research.uwa.edu.au)
	4. Short-Description
	The generic UWA lower body dynamic bodybuilder model parameter file
	5. Notes
	Mp file is the same as the previous versions. Name changed to match new model. 
	6. Modification-History
	Version  Author     Date                 Change
	1.0       DC    30/June/2016         No content change
*}

{* Length Mandatory *}
 {*General Unitless Mandatory*}

{* if female, type 1, if male, type 0*}
 {*General Unitless Mandatory*}

{* if Adult, type 1, if child, type 0*}
  {*General Unitless Mandatory*}

{* Use this to define the global anatomical CS based on subject progression *}
 {*General Unitless Mandatory*}

{* Subject Specific Measurements including Manual Input From Foot Rig *}


{* Subject Specific MManual Input From Foot Rig *}
{* Rear foot static eversion measure: calcaneus relative to vertical *}


{* If Optimal Hip, Helical Knee or Helical Elbow Found in Matlab, then type 1 *}
$OptimalHip = 0 {* General Unitless Mandatory *}
$HelicalKnee = 0 {* General Unitless Mandatory*}


{* Optional Inputs and Matlab Optimal Joint Inputs*}
$LOptimalHip = {0,0,0} {* Left Hip Length Optional*}
$ROptimalHip = {0,0,0} {* Right Hip Length Optional*}
$RKHA1 = {0,0,0} {* Right Knee Length Optional*}
$RKHA2 = {0,0,0} {* Right Knee Length Optional*}
$LKHA1 = {0,0,0}  {* Left Knee Length Optional*}
$LKHA2 = {0,0,0}  {* Left Knee Length Optional*}

{* Subject Specific Parameter Information Output by Matlab or Static Model Processing*}
{* ----------------------------------------------------------------------------------*}

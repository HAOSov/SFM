

#created with Auto-Rigger v1.8 for biped models
#BY http://steamcommunity.com/id/OMGTheresABearInMyOatmeal/
#This is just a modified version of valves' biped simple script.

import vs
import random
sfm.EndRig()
#==================================================================================================
def AddValidObjectToList( objectList, obj ):
    if ( obj != None ): objectList.append( obj )


#==================================================================================================
def HideControlGroups( rig, rootGroup, *groupNames ):
    for name in groupNames:
        group = rootGroup.FindChildByName( name, False )
        if ( group != None ):
            rig.HideControlGroup( group )





def CreateOrientConstraint( target, slave, bCreateControls=True, group=None ) :
    ''' Method for creating a single target orient constraint '''
    
    if ( target == None ):
        return

    targetDag = sfmUtils.GetDagFromNameOrObject( target )
    slaveDag = sfmUtils.GetDagFromNameOrObject( slave )
    
    sfm.PushSelection()
    sfmUtils.SelectDagList( [ targetDag, slaveDag ] )
    
    
    orientConstraintTarget = sfm.OrientConstraint( controls=bCreateControls )
    
    if ( group != None ):

        if ( orientConstraintTarget != None ):
            orientWeightControl = orientConstraintTarget.FindWeightControl()
            if ( orientWeightControl != None ):
                group.AddControl( orientWeightControl )
            
    sfm.PopSelection()
    return






#==================================================================================================
# Create the reverse foot control and operators for the foot on the specified side
#==================================================================================================
def CreateReverseFoot( controlName, sideName, gameModel, animSet, shot, helperControlGroup, footControlGroup,footroll ) :

    # Cannot create foot controls without heel position, so check for that first
    #heelAttachName = "pvt_heel_" + sideName
    if (  footroll == None ):
        #print "Could not create foot control " + controlName + ", model is missing heel attachment point: " + heelAttachName;
        return None

    footRollDefault = 0.5
    if (footroll=="x"):
        rotationAxis = vs.Vector( 1, 0, 0 )
    elif (footroll=="y"):
        rotationAxis = vs.Vector( 0, 1, 0 )
    elif (footroll=="z"):
        rotationAxis = vs.Vector( 0, 0, 1 )
    else: return None
    # Construct the name of the dag nodes of the foot and toe for the specified side
    footName = "rig_foot_" + sideName
    toeName = "rig_toe_" + sideName

    # Get the world space position and orientation of the foot and toe
    footPos = sfm.GetPosition( footName )
    footRot = sfm.GetRotation( footName )
    toePos = sfm.GetPosition( toeName )

    # Setup the reverse foot hierarchy such that the foot is the parent of all the foot transforms, the
    # reverse heel is the parent of the heel, so it can be used for rotations around the ball of the
    # foot that will move the heel, the heel is the parent of the foot IK handle so that it can perform
    # rotations around the heel and move the foot IK handle, resulting in moving all the foot bones.
    # root
    #   + rig_foot_R
    #       + rig_knee_R
    #       + rig_reverseHeel_R
    #           + rig_heel_R
    #               + rig_footIK_R


    # Construct the reverse heel joint this will be used to rotate the heel around the toe, and as
    # such is positioned at the toe, but using the rotation of the foot which will be its parent,
    # so that it has no local rotation once parented to the foot.
    reverseHeelName = "rig_reverseHeel_" + sideName
    reverseHeelDag = sfm.CreateRigHandle( reverseHeelName, pos=toePos, rot=footRot, rotControl=False )
    sfmUtils.Parent( reverseHeelName, footName, vs.REPARENT_LOGS_OVERWRITE )



    # Construct the heel joint, this will be used to rotate the foot around the back of the heel so it
    # is created at the heel location (offset from the foot) and also given the rotation of its parent.
    heelName = "rig_heel_" + sideName
    #vecHeelPos = gameModel.ComputeAttachmentPosition( heelAttachName )
   # heelPos = [ vecHeelPos.x, vecHeelPos.y, vecHeelPos.z ]
    heelRot = sfm.GetRotation( reverseHeelName )
    heelDag = sfm.CreateRigHandle( heelName, pos=footPos, rot=heelRot, posControl=True, rotControl=False )
    sfmUtils.Parent( heelName, reverseHeelName, vs.REPARENT_LOGS_OVERWRITE )

    # Create the ik handle which will be used as the target for the ik chain for the leg
    ikHandleName = "rig_footIK_" + sideName
    ikHandleDag = sfmUtils.CreateHandleAt( ikHandleName, footName )
    sfmUtils.Parent( ikHandleName, heelName, vs.REPARENT_LOGS_OVERWRITE )

    # Create an orient constraint which causes the toe's orientation to match the foot's orientation
    footRollControlName = controlName + "_" + sideName
    toeOrientTarget = sfm.OrientConstraint( footName, toeName, mo=True, controls=False )
    footRollControl, footRollValue = sfmUtils.CreateControlledValue( footRollControlName, "value", vs.AT_FLOAT, footRollDefault, animSet, shot )

    # Create the expressions to re-map the footroll slider value for use in the constraint and rotation operators
    toeOrientExprName = "expr_toeOrientEnable_" + sideName
    toeOrientExpr = sfmUtils.CreateExpression( toeOrientExprName, "inrange( footRoll, 0.5001, 1.0 )", animSet )
    toeOrientExpr.SetValue( "footRoll", footRollDefault )

    toeRotateExprName = "expr_toeRotation_" + sideName
    toeRotateExpr = sfmUtils.CreateExpression( toeRotateExprName, "max( 0, (footRoll - 0.5) ) * 140", animSet )
    toeRotateExpr.SetValue( "footRoll", footRollDefault )

    heelRotateExprName = "expr_heelRotation_" + sideName
    heelRotateExpr = sfmUtils.CreateExpression( heelRotateExprName, "max( 0, (0.5 - footRoll) ) * -100", animSet )
    heelRotateExpr.SetValue( "footRoll", footRollDefault )

    # Create a connection from the footroll value to all of the expressions that require it
    footRollConnName = "conn_footRoll_" + sideName
    footRollConn = sfmUtils.CreateConnection( footRollConnName, footRollValue, "value", animSet )
    footRollConn.AddOutput( toeOrientExpr, "footRoll" )
    footRollConn.AddOutput( toeRotateExpr, "footRoll" )
    footRollConn.AddOutput( heelRotateExpr, "footRoll" )

    # Create the connection from the toe orientation enable expression to the target weight of the
    # toe orientation constraint, this will turn the constraint on an off based on the footRoll value
    toeOrientConnName = "conn_toeOrientExpr_" + sideName;
    toeOrientConn = sfmUtils.CreateConnection( toeOrientConnName, toeOrientExpr, "result", animSet )
    toeOrientConn.AddOutput( toeOrientTarget, "targetWeight" )

    # Create a rotation constraint to drive the toe rotation and connect its input to the
    # toe rotation expression and connect its output to the reverse heel dag's orientation
    toeRotateConstraintName = "rotationConstraint_toe_" + sideName
    toeRotateConstraint = sfmUtils.CreateRotationConstraint( toeRotateConstraintName, rotationAxis, reverseHeelDag, animSet )

    toeRotateExprConnName = "conn_toeRotateExpr_" + sideName
    toeRotateExprConn = sfmUtils.CreateConnection( toeRotateExprConnName, toeRotateExpr, "result", animSet )
    toeRotateExprConn.AddOutput( toeRotateConstraint, "rotations", 0 );

    # Create a rotation constraint to drive the heel rotation and connect its input to the
    # heel rotation expression and connect its output to the heel dag's orientation
    heelRotateConstraintName = "rotationConstraint_heel_" + sideName
    heelRotateConstraint = sfmUtils.CreateRotationConstraint( heelRotateConstraintName, rotationAxis, heelDag, animSet )

    heelRotateExprConnName = "conn_heelRotateExpr_" + sideName
    heelRotateExprConn = sfmUtils.CreateConnection( heelRotateExprConnName, heelRotateExpr, "result", animSet )
    heelRotateExprConn.AddOutput( heelRotateConstraint, "rotations", 0 )

    if ( helperControlGroup != None ):
        sfmUtils.AddDagControlsToGroup( helperControlGroup, reverseHeelDag, ikHandleDag, heelDag )

    if ( footControlGroup != None ):
        footControlGroup.AddControl( footRollControl )

    return ikHandleDag


#==================================================================================================
# Compute the direction from boneA to boneB
#==================================================================================================
def ComputeVectorBetweenBones( boneA, boneB, scaleFactor ):

    vPosA = vs.Vector( 0, 0, 0 )
    boneA.GetAbsPosition( vPosA )

    vPosB = vs.Vector( 0, 0, 0 )
    boneB.GetAbsPosition( vPosB )

    vDir = vs.Vector( 0, 0, 0 )
    vs.mathlib.VectorSubtract( vPosB, vPosA, vDir )
    vDir.NormalizeInPlace()

    vScaledDir = vs.Vector( 0, 0, 0 )
    vs.mathlib.VectorScale( vDir, scaleFactor, vScaledDir )

    return vScaledDir


#==================================================================================================
# Build a simple ik rig for the currently selected animation set
#==================================================================================================
def BuildRig(num,toe,collar,bone,footroll,fingerbones,handpreset):

    # Get the currently selected animation set and shot
    shot = sfm.GetCurrentShot()
    animSet = sfm.GetCurrentAnimationSet()
    gameModel = animSet.gameModel
    rootGroup = animSet.GetRootControlGroup()

    # Start the biped rig to which all of the controls and constraints will be added
    rig = sfm.BeginRig( "rig_biped_" + animSet.GetName() + str(random.randint(1,100)) );
    if ( rig == None ):
        return

    # Change the operation mode to passthrough so changes chan be made temporarily
    sfm.SetOperationMode( "Pass" )

    # Move everything into the reference pose
    sfm.SelectAll()
    sfm.SetReferencePose()

    #==============================================================================================
    # Find the dag nodes for all of the bones in the model which will be used by the script
    #==============================================================================================
    boneRoot      = sfmUtils.FindFirstDag( [ "RootTransform" ], True )
    bonePelvis    = sfmUtils.FindFirstDag( [ bone[0]  ], True )

    


    if num==4:
        
        boneSpine0    = sfmUtils.FindFirstDag( [  bone[1]  ], True )
        boneSpine1    = sfmUtils.FindFirstDag( [  bone[2]   ], True )
        boneSpine2    = sfmUtils.FindFirstDag( [  bone[3] ], True )
        boneSpine3    = sfmUtils.FindFirstDag( [  bone[4]   ], True )

        rigSpine0  = sfmUtils.CreateConstrainedHandle( "rig_spine_0",  boneSpine0,  bCreateControls=False )
        rigSpine1  = sfmUtils.CreateConstrainedHandle( "rig_spine_1",  boneSpine1,  bCreateControls=False )
        rigSpine2  = sfmUtils.CreateConstrainedHandle( "rig_spine_2",  boneSpine2,  bCreateControls=False )
        rigChest   = sfmUtils.CreateConstrainedHandle( "rig_chest",    boneSpine3,  bCreateControls=False )



        
    elif num==3:
        boneSpine0    = sfmUtils.FindFirstDag( [ bone[1]  ], True )
        boneSpine1    = sfmUtils.FindFirstDag( [ bone[2] ], True )
        boneSpine2    = sfmUtils.FindFirstDag( [ bone[3]], True )

        rigSpine0  = sfmUtils.CreateConstrainedHandle( "rig_spine_0",  boneSpine0,  bCreateControls=False )
        rigSpine1  = sfmUtils.CreateConstrainedHandle( "rig_spine_1",  boneSpine1,  bCreateControls=False )
        rigSpine2  = sfmUtils.CreateConstrainedHandle( "rig_spine_2",  boneSpine2,  bCreateControls=False )
        rigChest   = "null"


        


        
    elif num==2:
        boneSpine0    = sfmUtils.FindFirstDag( [ bone[1]  ], True )
        rigSpine0  = sfmUtils.CreateConstrainedHandle( "rig_spine_0",  boneSpine0,  bCreateControls=False )
        boneSpine1    = sfmUtils.FindFirstDag( [ bone[2]  ], True )
        rigSpine1  = sfmUtils.CreateConstrainedHandle( "rig_spine_1",  boneSpine1,  bCreateControls=False )
        rigSpine2  = "null"
        rigChest   = "null"
        
    elif num==1:
        boneSpine0    = sfmUtils.FindFirstDag( [ bone[1]  ], True )
        rigSpine0  = sfmUtils.CreateConstrainedHandle( "rig_spine_0",  boneSpine0,  bCreateControls=False )

        rigSpine1  = "null"
        rigSpine2  = "null"
        rigChest   = "null"
        
    boneNeck      = sfmUtils.FindFirstDag( [ bone[5] ], True )
    boneHead      = sfmUtils.FindFirstDag( [ bone[6]  ], True )
    boneUpperLegR = sfmUtils.FindFirstDag( [ bone[7]  ], True )
    boneLowerLegR = sfmUtils.FindFirstDag( [ bone[8] ], True )
    boneFootR     = sfmUtils.FindFirstDag( [ bone[9]  ], True )


    if toe:
        boneToeR      = sfmUtils.FindFirstDag( [ bone[10] ], True )
        boneToeL      = sfmUtils.FindFirstDag( [ bone[18] ], True )
        rigToeR    = sfmUtils.CreateConstrainedHandle( "rig_toe_R",    boneToeR,    bCreateControls=False )
        rigToeL    = sfmUtils.CreateConstrainedHandle( "rig_toe_L",    boneToeL,    bCreateControls=False )
    else:
        rigToeR = "null"
        rigToeL = "null"





    if collar:
        boneCollarR   = sfmUtils.FindFirstDag( [ bone[11] ], True )
        boneCollarL   = sfmUtils.FindFirstDag( [ bone[19] ], True )
        rigCollarR = sfmUtils.CreateConstrainedHandle( "rig_collar_R", boneCollarR, bCreateControls=False )
        rigCollarL = sfmUtils.CreateConstrainedHandle( "rig_collar_L", boneCollarL, bCreateControls=False )
    else:
        rigCollarR = "null"
        rigCollarL = "null"




    boneUpperArmR = sfmUtils.FindFirstDag( [ bone[12]  ], True )
    boneLowerArmR = sfmUtils.FindFirstDag( [ bone[13]  ], True )
    boneHandR     = sfmUtils.FindFirstDag( [ bone[14]  ], True )
    boneUpperLegL = sfmUtils.FindFirstDag( [ bone[15]  ], True )
    boneLowerLegL = sfmUtils.FindFirstDag( [ bone[16]  ], True )
    boneFootL     = sfmUtils.FindFirstDag( [ bone[17]  ], True )    
    boneUpperArmL = sfmUtils.FindFirstDag( [ bone[20]  ], True )
    boneLowerArmL = sfmUtils.FindFirstDag( [ bone[21]  ], True )
    boneHandL     = sfmUtils.FindFirstDag( [ bone[22]  ], True )


    #for fingers
    rig_finger_right=[]
    bone_finger_right=[]
    
    rig_finger_left=[]
    bone_finger_left=[]
    if (fingerbones):

        for i in fingerbones["right"]:
            bone_finger_right.append(sfmUtils.FindFirstDag( [ i ], True ))
            rig_finger_right.append(sfmUtils.CreateConstrainedHandle( "rig_"+i,     bone_finger_right[-1],    bCreateControls=False ))

        for i in fingerbones["left"]:
            bone_finger_left.append(sfmUtils.FindFirstDag( [ i ], True ))
            rig_finger_left.append(sfmUtils.CreateConstrainedHandle( "rig_"+i,     bone_finger_left[-1],    bCreateControls=False ))














    

    #==============================================================================================
    # Create the rig handles and constrain them to existing bones
    #==============================================================================================
    rigRoot    = sfmUtils.CreateConstrainedHandle( "rig_root",     boneRoot,    bCreateControls=False )
    rigPelvis  = sfmUtils.CreateConstrainedHandle( "rig_pelvis",   bonePelvis,  bCreateControls=False )
    rigNeck    = sfmUtils.CreateConstrainedHandle( "rig_neck",     boneNeck,    bCreateControls=False )
    rigHead    = sfmUtils.CreateConstrainedHandle( "rig_head",     boneHead,    bCreateControls=False )
    rigFootR   = sfmUtils.CreateConstrainedHandle( "rig_foot_R",   boneFootR,   bCreateControls=False )
    rigHandR   = sfmUtils.CreateConstrainedHandle( "rig_hand_R",   boneHandR,   bCreateControls=False )
    rigFootL   = sfmUtils.CreateConstrainedHandle( "rig_foot_L",   boneFootL,   bCreateControls=False )
    rigHandL   = sfmUtils.CreateConstrainedHandle( "rig_hand_L",   boneHandL,   bCreateControls=False )


    # Use the direction from the heel to the toe to compute the knee offsets,
    # this makes the knee offset indpendent of the inital orientation of the model.
    if toe:
        vKneeOffsetR = ComputeVectorBetweenBones( boneFootR, boneToeR, 10 )
        vKneeOffsetL = ComputeVectorBetweenBones( boneFootL, boneToeL, 10 )
    else:
        vKneeOffsetR = ComputeVectorBetweenBones( boneFootR, boneFootR, 10 )
        vKneeOffsetL = ComputeVectorBetweenBones( boneFootL, boneFootL, 10 )


    rigKneeR   = sfmUtils.CreateOffsetHandle( "rig_knee_R",  boneLowerLegR, vKneeOffsetR,  bCreateControls=False )
    rigKneeL   = sfmUtils.CreateOffsetHandle( "rig_knee_L",  boneLowerLegL, vKneeOffsetL,  bCreateControls=False )
    rigElbowR  = sfmUtils.CreateOffsetHandle( "rig_elbow_R", boneLowerArmR, -vKneeOffsetR,  bCreateControls=False )
    rigElbowL  = sfmUtils.CreateOffsetHandle( "rig_elbow_L", boneLowerArmL, -vKneeOffsetL,  bCreateControls=False )

    # Create a helper handle which will remain constrained to the each foot position that can be used for parenting.
    rigFootHelperR = sfmUtils.CreateConstrainedHandle( "rig_footHelper_R", boneFootR, bCreateControls=False )
    rigFootHelperL = sfmUtils.CreateConstrainedHandle( "rig_footHelper_L", boneFootL, bCreateControls=False )

    # Create a list of all of the rig dags
    allRigHandles=[]
    allRigHandles1 = [ rigRoot, rigPelvis, rigSpine0, rigSpine1, rigSpine2, rigChest, rigNeck, rigHead,
                      rigCollarR, rigElbowR, rigHandR, rigKneeR, rigFootR, rigToeR,
                      rigCollarL, rigElbowL, rigHandL, rigKneeL,rigFootL, rigToeL ];
    for i in allRigHandles1:
        if i !="null":
            allRigHandles.append(i)
    
    allRigHandles.extend(rig_finger_right+rig_finger_left)
    #==============================================================================================
    # Generate the world space logs for the rig handles and remove the constraints
    #==============================================================================================
    sfm.ClearSelection()
    sfmUtils.SelectDagList( allRigHandles )
    sfm.GenerateSamples()
    sfm.RemoveConstraints()


    #==============================================================================================
    # Build the rig handle hierarchy
    #==============================================================================================
    sfmUtils.ParentMaintainWorld( rigPelvis,        rigRoot )
    sfmUtils.ParentMaintainWorld( rigSpine0,        rigPelvis )


    if num==4:
        sfmUtils.ParentMaintainWorld( rigSpine1,        rigSpine0 )
        sfmUtils.ParentMaintainWorld( rigSpine2,        rigSpine1 )
        sfmUtils.ParentMaintainWorld( rigChest,         rigSpine2 )
        sfmUtils.ParentMaintainWorld( rigNeck,          rigChest )
        if collar:
            sfmUtils.ParentMaintainWorld( rigCollarR,       rigChest )
            sfmUtils.ParentMaintainWorld( rigCollarL,       rigChest )
        else:
            sfmUtils.ParentMaintainWorld( rigElbowR,       rigChest )
            sfmUtils.ParentMaintainWorld( rigElbowL,       rigChest )
            
    elif num==3:
        sfmUtils.ParentMaintainWorld( rigSpine1,        rigSpine0 )
        sfmUtils.ParentMaintainWorld( rigSpine2,        rigSpine1 )
        sfmUtils.ParentMaintainWorld( rigNeck,         rigSpine2 )

        if collar:
            sfmUtils.ParentMaintainWorld( rigCollarR,       rigSpine2 )
            sfmUtils.ParentMaintainWorld( rigCollarL,       rigSpine2 )
        else:
            sfmUtils.ParentMaintainWorld( rigElbowR,       rigSpine2 )
            sfmUtils.ParentMaintainWorld( rigElbowL,       rigSpine2 )


    elif num==2:
        sfmUtils.ParentMaintainWorld( rigSpine1,        rigSpine0 )
        sfmUtils.ParentMaintainWorld( rigNeck,        rigSpine1 )
          

        if collar:
            sfmUtils.ParentMaintainWorld( rigCollarR,       rigSpine1 )
            sfmUtils.ParentMaintainWorld( rigCollarL,       rigSpine1 )
        else:
            sfmUtils.ParentMaintainWorld( rigElbowR,       rigSpine1 )
            sfmUtils.ParentMaintainWorld( rigElbowL,       rigSpine1 )

            

    elif num==1:
        
        sfmUtils.ParentMaintainWorld( rigNeck,        rigSpine0 )
          

        if collar:
            sfmUtils.ParentMaintainWorld( rigCollarR,       rigSpine0 )
            sfmUtils.ParentMaintainWorld( rigCollarL,       rigSpine0 )
        else:
            sfmUtils.ParentMaintainWorld( rigElbowR,       rigSpine0 )
            sfmUtils.ParentMaintainWorld( rigElbowL,       rigSpine0 )

        
      
    sfmUtils.ParentMaintainWorld( rigHead,          rigNeck )
    sfmUtils.ParentMaintainWorld( rigFootHelperR,   rigRoot )
    sfmUtils.ParentMaintainWorld( rigFootHelperL,   rigRoot )
    sfmUtils.ParentMaintainWorld( rigFootR,         rigRoot )
    sfmUtils.ParentMaintainWorld( rigFootL,         rigRoot )



    
    sfmUtils.ParentMaintainWorld( rigKneeR,         rigFootR )
    sfmUtils.ParentMaintainWorld( rigKneeL,         rigFootL )
    if toe:
        sfmUtils.ParentMaintainWorld( rigToeR,          rigFootHelperR )
        sfmUtils.ParentMaintainWorld( rigToeL,          rigFootHelperL )

    if collar:
        sfmUtils.ParentMaintainWorld( rigElbowR,        rigCollarR )
        sfmUtils.ParentMaintainWorld( rigElbowL,        rigCollarL )
        
    sfmUtils.ParentMaintainWorld( rigHandR,	    rigRoot )


    sfmUtils.ParentMaintainWorld( rigHandL,	    rigRoot )





##############finger


    if (fingerbones):
        for i in range(len(rig_finger_right)):
            if(i%3==0):#base
                sfmUtils.ParentMaintainWorld( rig_finger_right[i],rigHandR )
            else:
                sfmUtils.ParentMaintainWorld( rig_finger_right[i],rig_finger_right[i-1] )


        for i in range(len(rig_finger_left)):
            if(i%3==0):#base
                sfmUtils.ParentMaintainWorld( rig_finger_left[i],rigHandL )
            else:
                sfmUtils.ParentMaintainWorld( rig_finger_left[i],rig_finger_left[i-1] )











    






    

    # Create the hips control, this allows a pelvis rotation that does not effect the spine,
    # it is only used for rotation so a position control is not created. Additionally add the
    # new control to the selection so the that set default call operates on it too.
    rigHips = sfmUtils.CreateHandleAt( "rig_hips", rigPelvis, False, True )
    sfmUtils.Parent( rigHips, rigPelvis, vs.REPARENT_LOGS_OVERWRITE )
    sfm.SelectDag( rigHips )

    # Set the defaults of the rig transforms to the current locations. Defaults are stored in local
    # space, so while the parent operation tries to preserve default values it is cleaner to just
    # set them once the final hierarchy is constructed.
    sfm.SetDefault()


    #==============================================================================================
    # Create the reverse foot controls for both the left and right foot
    #==============================================================================================
    rigLegsGroup = rootGroup.CreateControlGroup( "RigLegs" )
    rigHelpersGroup = rootGroup.CreateControlGroup( "RigHelpers" )
    rigHelpersGroup.SetVisible( False )
    rigHelpersGroup.SetSnappable( False )

    footIKTargetR = rigFootR
    footIkTargetL = rigFootL

    if ( gameModel != None ) :
        footRollIkTargetR = CreateReverseFoot( "rig_footRoll", "R", gameModel, animSet, shot, rigHelpersGroup, rigLegsGroup,footroll)
        footRollIkTargetL = CreateReverseFoot( "rig_footRoll", "L", gameModel, animSet, shot, rigHelpersGroup, rigLegsGroup,footroll )
        if ( footRollIkTargetR != None ) :
            footIKTargetR = footRollIkTargetR
        if ( footRollIkTargetL != None ) :
            footIkTargetL = footRollIkTargetL


    #==============================================================================================
    # Create constraints to drive the bone transforms using the rig handles
    #==============================================================================================

    # The following bones are simply constrained directly to a rig handle
    sfmUtils.CreatePointOrientConstraint( rigRoot,      boneRoot        )
    sfmUtils.CreatePointOrientConstraint( rigHips,      bonePelvis      )
    sfmUtils.CreatePointOrientConstraint( rigSpine0,    boneSpine0      )
    
    if num==4:
        sfmUtils.CreatePointOrientConstraint( rigSpine2,    boneSpine2      )
        sfmUtils.CreatePointOrientConstraint( rigChest,     boneSpine3      )
    elif num==3:
        sfmUtils.CreatePointOrientConstraint( rigSpine2,    boneSpine2      )
    elif num ==2:
        sfmUtils.CreatePointOrientConstraint( rigSpine1,    boneSpine1      )
        


        
    sfmUtils.CreatePointOrientConstraint( rigNeck,      boneNeck        )
    sfmUtils.CreatePointOrientConstraint( rigHead,      boneHead        )
    if collar:
        sfmUtils.CreatePointOrientConstraint( rigCollarR,   boneCollarR     )
        sfmUtils.CreatePointOrientConstraint( rigCollarL,   boneCollarL     )

    if toe:
        CreateOrientConstraint( rigToeR,      boneToeR        )
        CreateOrientConstraint( rigToeL,      boneToeL        )


##fingers
    if (fingerbones):
        for i in range(len(rig_finger_right)):
            CreateOrientConstraint( rig_finger_right[i],bone_finger_right[i] )


        for i in range(len(rig_finger_left)):
            CreateOrientConstraint( rig_finger_left[i],bone_finger_left[i] )









        

    # Create ik constraints for the arms and legs that will control the rotation of the hip / knee and
    # upper arm / elbow joints based on the position of the foot and hand respectively.



    
    sfmUtils.BuildArmLeg( rigKneeR,  footIKTargetR, boneUpperLegR,  boneFootR, True )
    sfmUtils.BuildArmLeg( rigKneeL,  footIkTargetL, boneUpperLegL,  boneFootL, True )
    sfmUtils.BuildArmLeg( rigElbowR, rigHandR,      boneUpperArmR,  boneHandR, True )
    sfmUtils.BuildArmLeg( rigElbowL, rigHandL,      boneUpperArmL,  boneHandL, True )


    #==============================================================================================
    # Create handles for the important attachment points
    #==============================================================================================
    attachmentGroup = rootGroup.CreateControlGroup( "Attachments" )
    attachmentGroup.SetVisible( False )

    sfmUtils.CreateAttachmentHandleInGroup( "pvt_heel_R",       attachmentGroup )
    sfmUtils.CreateAttachmentHandleInGroup( "pvt_toe_R",        attachmentGroup )
    sfmUtils.CreateAttachmentHandleInGroup( "pvt_outerFoot_R",  attachmentGroup )
    sfmUtils.CreateAttachmentHandleInGroup( "pvt_innerFoot_R",  attachmentGroup )

    sfmUtils.CreateAttachmentHandleInGroup( "pvt_heel_L",       attachmentGroup )
    sfmUtils.CreateAttachmentHandleInGroup( "pvt_toe_L",        attachmentGroup )
    sfmUtils.CreateAttachmentHandleInGroup( "pvt_outerFoot_L",  attachmentGroup )
    sfmUtils.CreateAttachmentHandleInGroup( "pvt_innerFoot_L",  attachmentGroup )



    #==============================================================================================
    # Re-organize the selection groups
    #==============================================================================================
    rigBodyGroup = rootGroup.CreateControlGroup( "RigBody" )
    rigArmsGroup = rootGroup.CreateControlGroup( "RigArms" )

    RightArmGroup = rootGroup.CreateControlGroup( "RightArm" )
    LeftArmGroup = rootGroup.CreateControlGroup( "LeftArm" )
    RightLegGroup = rootGroup.CreateControlGroup( "RightLeg" )
    LeftLegGroup = rootGroup.CreateControlGroup( "LeftLeg" )



    if num==4:
        sfmUtils.AddDagControlsToGroup( rigBodyGroup, rigRoot, rigPelvis, rigHips, rigSpine0, rigSpine1, rigSpine2, rigChest, rigNeck, rigHead )

    elif num==3:     
        sfmUtils.AddDagControlsToGroup( rigBodyGroup, rigRoot, rigPelvis, rigHips, rigSpine0, rigSpine1, rigSpine2, rigNeck, rigHead )


    elif num==2:
        sfmUtils.AddDagControlsToGroup( rigBodyGroup, rigRoot, rigPelvis, rigHips, rigSpine0, rigSpine1,rigNeck, rigHead )
    elif num==1:
        sfmUtils.AddDagControlsToGroup( rigBodyGroup, rigRoot, rigPelvis, rigHips, rigSpine0,rigNeck, rigHead )



    rigArmsGroup.AddChild( RightArmGroup )
    rigArmsGroup.AddChild( LeftArmGroup )

    if collar:
        sfmUtils.AddDagControlsToGroup( RightArmGroup,  rigHandR, rigElbowR, rigCollarR )
        sfmUtils.AddDagControlsToGroup( LeftArmGroup, rigHandL, rigElbowL, rigCollarL )
    else:

        sfmUtils.AddDagControlsToGroup( RightArmGroup,  rigHandR, rigElbowR )
        sfmUtils.AddDagControlsToGroup( LeftArmGroup, rigHandL, rigElbowL )        

    rigLegsGroup.AddChild( RightLegGroup )
    rigLegsGroup.AddChild( LeftLegGroup )

    if toe:
        sfmUtils.AddDagControlsToGroup( RightLegGroup, rigKneeR, rigFootR, rigToeR )
        sfmUtils.AddDagControlsToGroup( LeftLegGroup, rigKneeL, rigFootL, rigToeL )
    else:
        sfmUtils.AddDagControlsToGroup( RightLegGroup, rigKneeR, rigFootR )
        sfmUtils.AddDagControlsToGroup( LeftLegGroup, rigKneeL, rigFootL )        

    sfmUtils.MoveControlGroup( "rig_footRoll_L", rigLegsGroup, LeftLegGroup )
    sfmUtils.MoveControlGroup( "rig_footRoll_R", rigLegsGroup, RightLegGroup )



    sfmUtils.AddDagControlsToGroup( rigHelpersGroup, rigFootHelperR, rigFootHelperL )

    # Set the control group visiblity, this is done through the rig so it can track which
    # groups it hid, so they can be set back to being visible when the rig is detached.
    HideControlGroups( rig, rootGroup, "Body", "Arms", "Legs","Root" )

    #Re-order the groups

    rootGroup.MoveChildToBottom( rigBodyGroup )
    rootGroup.MoveChildToBottom( rigLegsGroup )
    rootGroup.MoveChildToBottom( rigArmsGroup )






    #==============================================================================================
    # Set the selection groups colors
    #==============================================================================================
    topLevelColor = vs.Color( 0, 128, 255, 255 )
    RightColor = vs.Color( 255, 0, 0, 255 )
    LeftColor = vs.Color( 0, 255, 0, 255 )

    rigBodyGroup.SetGroupColor( topLevelColor, False )
    rigArmsGroup.SetGroupColor( topLevelColor, False )
    rigLegsGroup.SetGroupColor( topLevelColor, False )
    attachmentGroup.SetGroupColor( topLevelColor, False )
    rigHelpersGroup.SetGroupColor( topLevelColor, False )

    RightArmGroup.SetGroupColor( RightColor, False )
    LeftArmGroup.SetGroupColor( LeftColor, False )
    RightLegGroup.SetGroupColor( RightColor, False )
    LeftLegGroup.SetGroupColor( LeftColor, False )






    if (fingerbones):
        HideControlGroups(rig, rootGroup,"Fingers")
        rightFingersGroup=rootGroup.CreateControlGroup( "Right_Fingers" )
        RightArmGroup.AddChild( rightFingersGroup )
        rightFingersGroup.SetGroupColor( RightColor, False )
        rightFingersGroup.SetSelectable( True )
        for rig in rig_finger_right:
            sfmUtils.AddDagControlsToGroup(rightFingersGroup,rig)

        leftFingersGroup=rootGroup.CreateControlGroup( "Left_Fingers" )
        LeftArmGroup.AddChild( leftFingersGroup )
        leftFingersGroup.SetGroupColor( LeftColor, False )
        leftFingersGroup.SetSelectable( True )
        for rig in rig_finger_left:
            sfmUtils.AddDagControlsToGroup(leftFingersGroup,rig)        
    else:
        

        
        fingersGroup = rootGroup.FindChildByName( "Fingers", False )
        rightFingersGroup = rootGroup.FindChildByName( "RightFingers", True )
        if ( rightFingersGroup != None ):
            RightArmGroup.AddChild( rightFingersGroup )
            rightFingersGroup.SetSelectable( True )

        leftFingersGroup = rootGroup.FindChildByName( "LeftFingers", True )
        if ( leftFingersGroup != None ):
            LeftArmGroup.AddChild( leftFingersGroup )
            leftFingersGroup.SetSelectable( True )



    # End the rig definition
    sfm.EndRig()
    return

#==================================================================================================
# Script entry
#==================================================================================================


boneList= [u'ValveBiped.Bip01_Pelvis', u'ValveBiped.Bip01_Spine', u'ValveBiped.Bip01_Spine1', u'ValveBiped.Bip01_Spine2', u'ValveBiped.Bip01_Spine4', u'ValveBiped.Bip01_Neck1', u'ValveBiped.Bip01_Head1', u'ValveBiped.Bip01_R_Thigh', u'ValveBiped.Bip01_R_Calf', u'ValveBiped.Bip01_R_Foot', None, None, u'ValveBiped.Bip01_R_UpperArm', u'ValveBiped.Bip01_R_Forearm', u'ValveBiped.Bip01_R_Hand', u'ValveBiped.Bip01_L_Thigh', u'ValveBiped.Bip01_L_Calf', u'ValveBiped.Bip01_L_Foot', None, None, u'ValveBiped.Bip01_L_UpperArm', u'ValveBiped.Bip01_L_Forearm', u'ValveBiped.Bip01_L_Hand'] 
fingerbones= {} 
BuildRig(4,False,False,boneList,None,fingerbones,None);

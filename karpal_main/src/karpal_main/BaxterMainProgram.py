#!/usr/bin/python

import time
import sys
import json
import roslib
import rospy

from RecipeParser import *
#from FindObject import *
from PickUpItem import *
#from MixItems import *
from karpal_moveit.karpal_moveit_api import *

if __name__ == "__main__":
    if sys.argv is not None and len(sys.argv) > 1:
        #rospy.init_node('baxter_main', anonymous=True)
        if sys.argv[1] is not None and sys.argv[1] == "sim":
            if sys.argv[1] is not None and sys.argv[1] == "test":
                testBothArm()
            else:
                #moveitLeft = karpal_moveit_api("left_arm")
                #moveitRight = karpal_moveit_api("right_arm")
                pickup("Bowl")
                # moveitLeft.addObject()
    
                # moveitLeft.group.set_named_target('left_neutral')
                # moveitLeft.group.go()
                # rospy.sleep(1)
    
                # moveitRight.group.set_named_target('right_neutral')
                # moveitRight.group.go()
                # rospy.sleep(1)
    
                # milkPos = [0.72, 0.28, -0.055]
                # bowlPos = [0.84, 0, -0.07]
                # cerealPos = [0.88, 0.38, -0.055]
    
                # mixerPos = [0.79, -0.40, 0.12]
                # mixerPickOrientation = [-0.178, 0.686, 0.157, 0.687]
    
                # picked = moveitLeft.pick(milkPos, preGraspDistance = 0.08, fixedOrientation = False)
    
                # if(picked):
                #     if (moveitLeft.pour(bowlPos)):
                #         moveitLeft.place(milkPos)
    
                # rospy.sleep(3)
                # pose = moveitLeft.moveHelper.get_current_pose()
                # pose.position.x -= 0.08
                # pose.position.y += 0.08
                # moveitLeft.move_to_target_pose(pose)
    
                # rospy.sleep(1)
                # moveitLeft.group.set_named_target('left_neutral')
                # moveitLeft.group.go()
                # rospy.sleep(1)
    
                # pickOrientation = [0.278, 0.678, -0.19, 0.653]
    
                # picked = moveitLeft.pick(cerealPos)
    
                # if(picked):
                #     if (moveitLeft.pour(bowlPos)):
                #         moveitLeft.place(cerealPos)

        else:
            print "Opening " + sys.argv[1]

            fileLocation = sys.argv[1]
        
            f = open(fileLocation)
            data = f.read()

            parser = RecipeParser()
            parser.parse(data)
            recipeName = parser.recipeName
            ings = parser.recipeIngredients
            tools = parser.recipeTools
            steps = parser.recipeSteps

            actionParser = ActionParser(
                parser.recipeSteps, parser.recipeIngredients, parser.recipeTools)
            actionParser.parse()
            actions = actionParser.actions

            for x in actions:
                print "Step {}:".format(x.id)

                for preAction in x.precursors:
                    if (preAction.name == "FIND"):
                        print(
                            "Call the find object service to invoke the finding")
                        # findObjectClient(preAction.item)
                    elif (preAction.name == "GRASP"):
                        print("Call the pickup item service")
                        pickup(preAction.item)

                    elif (preAction.name == "PLACE"):
                        print("Call the place item service")
                    elif(preAction.name == "POUR"):
                        print("Call the pour item service")
                    elif(preAction.name == "STIR"):
                        print("Call the stir service")
                    elif(preAction.name == "RETURN"):
                        print("Call the return item service")

                        print preAction.name or None
                        print preAction.item
                        print ""

                        print ""

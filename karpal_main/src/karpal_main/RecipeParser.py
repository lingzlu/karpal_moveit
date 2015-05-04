#!/usr/bin/python
 
import time
import sys
import json
#import roslib
#import rospy


## @package RecipeParser
#  Parser to parse recipe information from input

## ItemLocation class
#  Class object that stores information about an item location with x, y, z cordinate
class ItemLocation:
    ## Constructor
    #  @param self The object pointer
    #  @param _x The x cordinate
    #  @param _y The y cordinate
    #  @param _z The z cordinate
    def __init__(self, _x = 0.0, _y = 0.0, _z = 0.0):
        self.x = _x
        self.y = _y
        self.z = _z
 
## Recipe class
#  Class object that stores information about a recipe, including its name, the list of ingredients, the tools, and the cooking instructions
class Recipe:
    ## Constructor
    #  @param self The object pointer
    #  @param name The name of the recipe
    def __init__(self, _name):
        self.name = _name
        self.ingredients = []
        self.tools = []
        self.steps = []
 
## Ingredient class
#  Class object that stores information about the ingredient, include its name, its quantity, and its location
class Ingredient:
    ## Constructor
    #  @param self The object pointer
    def __init__(self):
        self.name = ""
        self.quantity = ""
        self.location = None
 

## Tool class
#  Class object that stores information about the tool that needed for cooking, including its name and location
class Tool:
    ## Constructor
    #  @param sef The object pointer
    def __init__(self):
        self.name = ""
        self.location = None
 
## Step class
#  Class object that stores information about the cooking instruction
class Step:
    ## Constructor
    #  @param self The object pointer
    def __init__(self):
        self.id = 0
        self.description = ""
 
## RecipeParser class
#  Class object that handle the parsing information of the recipe
class RecipeParser:
    ## Constructor
    #  @param self The object pointer
    def __init__(self):
        self.data = ""
        self.recipeName = ""
        self.recipeIngredients = []
        self.recipeTools = []
        self.recipeSteps = []
 
    ## Parse the recipe
    #  @param self The object pointer
    #  @param _data The recipe data from the recipe input file
    def parse(self, _data):
        self.data = json.loads(_data)
 
        self.recipeName = self.data["name"]
 
        _recipeIngredients = self.data["ingredients"]
        _recipeTools = self.data["tools"]
        _recipeSteps = self.data["steps"]
 
        for idx, val in enumerate(_recipeIngredients):
            ing = Ingredient()
            ing.name = val["name"]
            ing.quantity = val["quantity"]
            ingredientLocation = ItemLocation(val["location"]["x"], val["location"]["y"], val["location"]["z"])
            ing.location = ingredientLocation
             
            self.recipeIngredients.append(ing)

        for idx, val in enumerate(_recipeTools):
            tool = Tool()
            tool.name = val["name"]
            toolLocation = ItemLocation(val["location"]["x"], val["location"]["y"], val["location"]["z"])
            tool.location = toolLocation

            self.recipeTools.append(tool)

        for idx, val in enumerate(_recipeSteps):
            step = Step()
            step.id = idx + 1
            step.description = val

            self.recipeSteps.append(step)
## Variables for primitive actions
PRIMITIVE_ACTIONS = ["POUR", "STIR", "MIX", "PLACE", "BAKE", "COOK", "ADD", "PICK"]
## Variables for other actions
OTHER_ACTIONS = ["MOVE", "FIND", "IDENITFY", "GRASP"]

## SubAction class
#  Class object that stores information about the sub-action
class SubAction:
    ## Constructor
    #  @param self The object pointer
    #  @param _name The name of the sub-action. Default to empty.
    #  @param _item The item that needed to be performed by the sub-action. Default to empty.
    def __init__(self, _name = "", _item = ""):
        self.name = _name
        self.item = _item

## Action class
#  Class object that stores information about the action
class Action:
    ## Constructor
    #  @param self The object pointer
    def __init__(self):
        self.id = 0
        self.name = ""
        self.precursors = []
        self.items = []
        self.ings = []

    ## Find the list of precursor actions for the primitive action
    #  @param self The object pointer
    def findPrecusors(self):
        if self.name != "":
            currentAction = self.name.upper()
            if(currentAction in PRIMITIVE_ACTIONS):
                if currentAction == "MIX":
                    self.precursors.append(SubAction("GRASP", "Bowl"))
                    self.precursors.append(SubAction("PLACE", "Bowl"))

                    for item in self.items:
                        self.precursors.append(SubAction("GRASP", item))
                        self.precursors.append(SubAction("POUR", item))
                        self.precursors.append(SubAction("RETURN", item))

                elif currentAction == "STIR":
                    self.precursors.append(SubAction("GRASP", "Spoon"))
                    self.precursors.append(SubAction("GRASP", "Bowl"))
                    self.precursors.append(SubAction("STIR", "Bowl"))
                    self.precursors.append(SubAction("RETURN", "Spoon"))

                elif currentAction == "PLACE":
                    for item in self.items:
                        self.precursors.append(SubAction("RELEASE", item))

                elif currentAction == "PICK":
                     for item in self.items:
                        self.precursors.append(SubAction("GRASP", item))
                        
                elif currentAction == "POUR" or currentAction == "ADD":
                    for item in self.items:
                        self.precursors.append(SubAction("GRASP", item))

                elif currentAction == "BAKE" or currentAction == "COOK":
                    for item in self.items:
                        self.precursors.append(SubAction("GRASP", item))

    ## Find the ingredients that are needed for the particular action
    #  @param self The object pointer
    #  @param input The instruction cooking excluding the primitive action
    #  @param ings The list of ingredients
    def findItems(self, input, ings):
        if input != None and input != "" and ings != None and ings != []:
            for idx, val in enumerate(ings):
                if val.name.lower() in input.lower():
                    self.items.append(val.name)
                    
    def findObjectsPrecursor(self, ings, tools):
        for idx, val in enumerate(tools):
            self.precursors.append(SubAction("FIND", val.name))
        
        for idx, val in enumerate(ings):
            self.precursors.append(SubAction("FIND", val.name))

## ActionParser class
#  Class object that handles the parsing of the primitive action and its precusor actions
class ActionParser:
    ## Constructor
    #  @param self The object pointer
    #  @param _steps The cooking instructions
    #  @param _ingredients The list of ingredients
    #  @param _tools The list of cooking tools
    def __init__(self, _steps, _ingredients, _tools):
        self.steps = _steps
        self.actions = []
        self.ings = _ingredients
        self.tools = _tools
        
    ## Parse the actions
    #  @param self The object pointer
    def parse(self):
        initialAction = Action()
        initialAction.id = 0
        initialAction.name = "IDENTIFY"
        initialAction.ings = self.ings
        initialAction.findObjectsPrecursor(self.ings, self.tools)
        self.actions.append(initialAction   )
        
        for idx, val in enumerate(self.steps):
            direction = val.description
            index = direction.find(' ')
            if index != -1:
                primitiveAction = direction[:index]
                action = Action()
                action.id = val.id
                action.name = primitiveAction
                action.ings = self.ings
                action.findItems(direction[index + 1:], self.ings)
                action.findPrecusors()
                

                self.actions.append(action)
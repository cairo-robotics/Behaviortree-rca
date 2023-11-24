import openai, os, re
import copy
from abc import ABC
from comment_parser import comment_parser
import docstring_parser

from langchain.chat_models import ChatOpenAI
from langchain.prompts import ChatPromptTemplate

class Bot(ABC):
    def __init__(self) -> None:
        super().__init__()
        self.API_KEY        =  os.environ['OPEN_AI_API_KEY']
        self.docstringParser=  docstring_parser.google.GoogleParser()
        
    def extract_docstrings_from_file(self, file_path):
        with open(file_path, 'r') as file:
            code = file.read()

        docstrings = []

        # Regular expression pattern to match docstrings
        pattern = r'(["\']{3})([\s\S]*?)(["\']{3})'
        matches = re.findall(pattern, code)


        keys    = [match[1].strip().split(':')[0] for match in matches]
        vals    = [self.docstringParser.parse(match[1].strip()) for match in matches]
    
        return keys, vals
   
    def create_data(self, ClientLoc: str, ServerLoc: str, BehaviorTreeLoc: str) -> None:
        """Creates a dictionary for client, server code comments/description for different functions and classes,
        to be used later for prompt generation

        Args:
            ClientLoc (str): Location of client file(.cpp)
            ServerLoc (str): Location of server file(.py)
        """
        clientComments = comment_parser.extract_comments(ClientLoc)
        
        ClientClasses_keys = [str((i.text().split('\n')[1]).split(':')[0]).split('*')[1] for i in clientComments if ('*' in i.text()) and ('class' in i.text())]
        ClientClasses_vals = [str((i.text().split('\n')[1]).split(':')[1]).split(',')[0] for i in clientComments if ('*' in i.text()) and ('class' in i.text())]
        
        self.ClientClasses_dict = dict(zip(ClientClasses_keys, ClientClasses_vals))
        
        ClientTicks_vals   = [str((i.text().split('\n')[1]).split(':')[1]).split(',')[0] for i in clientComments if ('*' in i.text()) and ('tick function' in i.text())]
        
        self.ClientTicks_dict   = dict(zip(ClientClasses_keys, ClientTicks_vals))
        
        ServerComments_keys, ServerComments_vals = self.extract_docstrings_from_file(ServerLoc)
        
        self.ServerFunctions_dict = dict(zip(ServerComments_keys, ServerComments_vals))
        
        with open(BehaviorTreeLoc, 'r') as file:
            self.BTxml = file.read()
        
    def create_message(self, base_msg = ""):
        """Creates a message by taking roslogs, from server, errors from execution, 
        and function comments explaining how the code works to the LLM.
        This is version 1 of this function later it will take these, output from GPT 
        and generate a prompt from a smaller LLMs using the previously mentioned 
        """
        
        if len(base_msg) == 0:
            self.message = self.init_message()
        else:
            self.message = base_msg
            
        serverCode = "".join([self.ServerFunctions_dict[t].short_description + " " for t in self.ServerFunctions_dict])
        
        clientCode = "".join([(t + ": " + self.ClientTicks_dict[t] + " \n") for t in self.ClientTicks_dict])
        
        self.message += f"The server side code's function description can be given as: \n {serverCode} \n \n The Client side code function descriptions can be given as: \n {clientCode} \n \n The behavior tree responsible for execution of these functions is given as: {self.BTxml}"
        
    
    def init_message(self):
        return "You're an AI helper assigned with the job of debugging an autonomous pick and place system developed using BehaviorTree.CPP. \
            To briefly describe the enviornment, we have a assembly board in which the specific things are to be placed and, \
            we have a mat which has locations where the individual parts are placed initially. Our robot arm(Sawyer) picks the ket from the mat and \
            places that in a hole in the assembly board. Our robot arm is equipped with a gripper that picks the ket,\
            and an RGBD camera that calculates the pick position of the ket. The software can be broken down into server and client.\
            Server is responsible for the code that interacts with the robot hardware using intera API that the OEM provides.\
            Client software as mentioned before is built using BehaviorTree.CPP, the job of this is to organise low level functionalities into a heirarchichal system that is well abstracted and individual processes are isolated from each other."
    
    def call_gpt(self) -> str:
        chat = openai.ChatCompletion.create(
            model="gpt-3.5-turbo", messages=self.messages
        )
        reply = chat.choices[0].message.content

def test():
    tt = Bot()
    tt.create_data("../../pick_and_place/src/BTClient.cpp", "../../pick_and_place/scripts/BTNodeServer.py", "../../pick_and_place/src/temp.xml")
    tt.create_message()
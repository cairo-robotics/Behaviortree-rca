import openai, os, re
import copy
from abc import ABC
from comment_parser import comment_parser
import docstring_parser

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
   
    def create_data(self, ClientLoc: str, ServerLoc: str) -> None:
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
        
    def create_message(self):
        """Creates a message by taking roslogs, from server, errors from execution, 
        and function comments explaining how the code works to the LLM.
        This is version 1 of this function later it will take these, output from GPT 
        and generate a prompt from a smaller LLMs using the previously mentioned 
        """
        
        self.message = ""
        
        pass
    
    def call_gpt(self) -> str:
        chat = openai.ChatCompletion.create(
            model="gpt-3.5-turbo", messages=self.messages
        )
        reply = chat.choices[0].message.content

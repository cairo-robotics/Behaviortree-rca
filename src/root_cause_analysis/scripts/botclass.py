#https://medium.com/@reddyyashu20/gpt-3-embeddings-perform-text-similarity-semantic-search-classification-and-clustering-part-4-abbcf447faf8

import openai, os, re
import pandas as pd
from colorama import Fore, Style
from sklearn.model_selection import train_test_split
from sklearn.cluster import KMeans
import numpy as np
import copy
from abc import ABC
from comment_parser import comment_parser
import docstring_parser

from langchain.llms import OpenAI
from langchain.chat_models import ChatOpenAI
from langchain.memory import ConversationBufferMemory
from langchain.chains import ConversationChain


class Bot(ABC):
    def __init__(self) -> None:
        super().__init__()
        self.API_KEY            =  os.environ['OPEN_AI_API_KEY']
        self.docstringParser    =  docstring_parser.google.GoogleParser()
        self.memory             =  ConversationBufferMemory()
        self.llm                =  ChatOpenAI(model='gpt-3.5-turbo')
        self.conversation       =  ConversationChain(
                                        llm=self.llm,
                                        memory=self.memory
                                        )
        
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
            BehaviorTreeLoc (str): Location of Behaviortree.CPP xml file(.xml)
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
        
    def init_message(self, base_msg = ""):
        """Creates a message by taking roslogs, from server, errors from execution, 
        and function comments explaining how the code works to the LLM.
        This is version 1 of this function later it will take these, output from GPT 
        and generate a prompt from a smaller LLMs using the previously mentioned 
        """
        
        self.message = "You're an AI helper assigned with the job of debugging an autonomous pick and place system developed using BehaviorTree.CPP. \
            To briefly describe the enviornment, we have a assembly board in which the specific things are to be placed and, \
            we have a mat which has locations where the individual parts are placed initially. Our robot arm(Sawyer) picks the ket from the mat and \
            places that in a hole in the assembly board. Our robot arm is equipped with a gripper that picks the ket,\
            and an RGBD camera that calculates the pick position of the ket. The software can be broken down into server and client.\
            Server is responsible for the code that interacts with the robot hardware using intera API that the OEM provides.\
            Client software as mentioned before is built using BehaviorTree.CPP, the job of this is to organise low level functionalities into a heirarchichal system that is well abstracted and individual processes are isolated from each other. Behavior tree's xml is the heirarchy in which the nodes are called, different nodes have different meanings."
            
        serverCode = "".join([self.ServerFunctions_dict[t].short_description + " " for t in self.ServerFunctions_dict])
        
        clientCode = "".join([(t + ": " + self.ClientTicks_dict[t] + " \n") for t in self.ClientTicks_dict])
        
        self.message += f"The server side code's function description can be given as: \n {serverCode} \n \n The Client side code function descriptions can be given as: \n {clientCode} \n \n The behavior tree responsible for execution of these functions is given as: {self.BTxml}"


    def chatbot_prompt(self):
        self.message += "\nGiven this information, you're now given a human user who has observed the entire experiment happen, by taking inputs from the user you have to perform a root cause analysis on the behaviortree and suggest changes in a final report. You can ask the user questions in one by one manner, you can start by asking how did the experiment go, followed by subsequent questions that you think will be helpful for debugging the system."      
        
    def cluster_prompt(self, message):
        embedding =  openai.Embedding.create(input=message, engine="text-similarity-davinci-001")
        name      =  self.clf.predict(embedding)
        return name
    
    def bot_loop(self):
        self.message = ""
        
        self.init_message()
        
        self.chatbot_prompt()
        # Reply after conditioning GPT to be a chat agent
        reply = self.conversation.predict(input=self.message)
        print(reply)
        
        while True:
            self.message = ""
            self.message = f"The log for node {self.mapping[predict]} is {self.nodes_logs[self.mapping[predict]]} and the description for the same is {self.nodes_description[self.mapping[predict]]}, I hope this answers the query about the previous doubt, the user may not be aware exactly but this is what the logs say. User's response about real world observation: "
            self.message += input("Enter Your Response: ")
            reply = self.conversation.predict(input=self.message)
            embd      = openai.Embedding.create(input=reply,
                                                engine="text-similarity-davinci-001")
            embd      = embd/np.linalg.norm(embd, axis=1, keepdims=True)
            predict   = self.clustering_model.predict(embd)
            print(Fore.GREEN + reply + Style.RESET_ALL)


class trainClassifier():
    def __init__(self) -> None:
        self.dataframe = None
        self.test_size = 0.2
        self.random_state = 42
        self.classifier = None
        self.nodes = []
        self.client = OpenAI()

    
    def loadData(self) -> None:
        self.dataframe = pd.read_csv('classificationTraining.csv')
        self.clustering_model = KMeans(n_clusters=len(self.nodes))
        self.clustering_model.fit(self.dataframe.davinci_similarity.values)
        self.mapping = dict(list(set([(self.clustering_model.labels_[i], self.dataframe.Label.value[i]) for i in range(len(self.clustering_model.labels_))])))

    def createData(self, xmlPath : str) -> None:
        count = 5
        self.nodes = self.loadNodes(xmlPath)
        
        for i in self.nodes:
            response  = self.client.chat.completions.create(
                                                model="gpt-3.5-turbo",
                                                messages=[{"role": "user", "content": f"Write {count} questions about the behaviortree node {i} and write them inside <li> tags so that it is easy to parse."}])
            
            data1     = data1 + (self.isolate_description(response))
            embd      = openai.Embedding.create(input=data1,
                                                engine="text-similarity-davinci-001")
            resp_embeddings = resp_embeddings  +  (embd/np.linalg.norm(embd, axis=1, keepdims=True))
            
            label     = label + [i for _ in range(count)]
            
        self.dataframe = pd.DataFrame({'Label' : label, 'Text' : data1, 'davinci_similarity' : resp_embeddings})
        self.dataframe.to_csv('classificationTraining.csv', index=False)
        
    

def test():
    tt = Bot()
    tt.create_data("../../pick_and_place/src/BTClient.cpp", "../../pick_and_place/scripts/BTNodeServer.py", "../../pick_and_place/src/temp.xml")
    tt.bot_loop()
    
test()
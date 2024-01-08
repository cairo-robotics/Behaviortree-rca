import openai, os, re
import pandas as pd
from colorama import Fore, Style
from sklearn.cluster import KMeans
from sklearn.manifold import TSNE
import matplotlib.pyplot as plt
import seaborn as sns
from statistics import mode
import pickle
import json
import shutil

from queue import Queue
from lxml import etree
from parse import parse
import numpy as np
import copy
from abc import ABC
from comment_parser import comment_parser
import docstring_parser
from bs4 import BeautifulSoup
from enum import Enum
from pathlib import Path


from langchain.llms import OpenAI
from langchain.chat_models import ChatOpenAI
from langchain.memory import ConversationBufferMemory
from langchain.chains import ConversationChain


class trainClassifier():
    def __init__(self) -> None:
        self.dataframe = None
        self.test_size = 0.2
        self.random_state = 42
        self.classifier = None
        self.nodes = []
        self.client = OpenAI()
        self.count = 20

    def loadLabelsAndModel(self, modelPath: str, labelsPath: str) -> None:
        with open(modelPath, 'rb') as f:
            self.clustering_model = pickle.load(f)

        with open(labelsPath, 'r') as f:
            self.mapping = json.load(f)

    def loadData(self, xmlPath : str) -> None:
        self.dataframe = pd.read_pickle('classificationTraining.pkl')
        self.nodes     = []
        # Saves unique items in a list with relative order maintained.
        for item in self.dataframe.Label.values:
            if item not in self.nodes:
                self.nodes.append(item)

        X = np.array([i.astype(np.float64) for i in self.dataframe.davinci_similarity.values],dtype=np.float64)

        self.clustering_model = KMeans(n_clusters=len(self.nodes))
        tsne = TSNE(random_state=0, n_iter=10000)
        tsne_results = tsne.fit_transform(X)
        df_tsne = pd.DataFrame(tsne_results, columns=['TSNE1', 'TSNE2'])
        fig, ax = plt.subplots(figsize=(8,6)) # Set figsize
        sns.set_style('darkgrid', {"grid.color": ".6", "grid.linestyle": ":"})
        sns.scatterplot(data=df_tsne, x='TSNE1', y='TSNE2')
        plt.title('Scatter plot of news using t-SNE')
        plt.xlabel('TSNE1')
        plt.ylabel('TSNE2')
        plt.axis('equal')
        plt.show()

        self.clustering_model.fit(X)
        [print(f"Clustering of data result in labels for debugging. iter: {int(i/self.count)} \n \n", self.clustering_model.labels_[i:i+self.count], "\n") for i in np.arange(0,len(self.clustering_model.labels_),self.count)]
        self.mapping = dict(list(set([(int(mode(self.clustering_model.labels_[i:i+self.count])), self.nodes[int(i/self.count)]) for i in np.arange(0, len(self.clustering_model.labels_), self.count)])))
        with open('QuestionClassificationModel.pkl','wb') as f:
            pickle.dump(self.clustering_model,f)
        with open("ClassificationLabels.json", "w") as f:
            json.dump(self.mapping, f, indent=4)        

    def isolate_description(self, response : str) -> list:
        soup = BeautifulSoup(response, 'html.parser')
        li_texts = [li.text for li in soup.find_all('li')]
        return li_texts 

    def loadNodes(self, xmlPath : str) -> dict:
        with open(xmlPath, 'r') as file:
            BTxml = file.read()

        parser = etree.XMLParser(recover=True)
        tree = etree.fromstring(BTxml, parser=parser)

        qq = Queue()

        qq.put(tree)

        node_list = []

        while not qq.empty():
            # BFS to access all the xml trees' node and store the unique ones
            node = qq.get()
            if not (node.tag == 'root' or node.tag == 'BehaviorTree' or node.tag == 'Sequence'):
                node_list.append((node.tag, '' if len(node.values()) == 0 else node.values()[0]))
            print(node.tag)
            for i in node.getchildren():
                qq.put(i)
        return dict(list(set(node_list)))

    def createData(self, xmlPath : str) -> None:
        self.nodes = self.loadNodes(xmlPath)
        data1 = []
        resp_embeddings = []
        label = []
        for i in self.nodes:
            response  = openai.ChatCompletion.create(
                                                model="gpt-3.5-turbo",
                                                messages=[{"role": "user", "content": f"Write {self.count} unique questions about the behaviortree node {i} that uniquely capture the action that this behaviortree node does and, write them inside <li> tags so that it is easy to parse."}])
            ques      = self.isolate_description(response.choices[0].message.content)
            data1     = data1 + (ques)
            embd      = openai.Embedding.create(input=ques, engine="text-similarity-babbage-001")
            resp_embeddings = resp_embeddings  +  [(i.embedding/np.linalg.norm(i.embedding, axis=0, keepdims=True)) for i in embd.data] 
            label     = label + [i for _ in range(self.count)]
            print(label, data1, len(data1), len(resp_embeddings))

        self.dataframe = pd.DataFrame({'Label' : label, 'Text' : data1, 'davinci_similarity' : resp_embeddings})
        self.dataframe.to_pickle('classificationTraining.pkl')

class Bot(ABC):
    def __init__(self) -> None:
        super().__init__()
        self.API_KEY            =  os.environ['OPENAI_API_KEY']
        self.docstringParser    =  docstring_parser.google.GoogleParser()
        self.memory             =  ConversationBufferMemory()
        self.llm                =  ChatOpenAI(model='gpt-3.5-turbo')
        self.conversation       =  ConversationChain(
                                        llm=self.llm,
                                        memory=self.memory
                                        )
        self.nodes_logs         =  None

    def extract_docstrings_from_file(self, file_path):
        """_summary_

        Args:
            file_path (_type_): _description_

        Returns:
            _type_: _description_
        """
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
        """Creates a dictionary for client, 
        server code comments/description for different functions and classes,
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

    def init_message(self):
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

    def cluster_prompt(self, message : str, textClassifier : trainClassifier) -> None:
        embd      = openai.Embedding.create(input=message, engine="text-similarity-babbage-001")
        embd      = embd.data[0].embedding/np.linalg.norm(embd.data[0].embedding,
                                                          axis=0,
                                                          keepdims=True).reshape(1, -1)
        predict   = textClassifier.clustering_model.predict(embd)
        return predict

    def load_logs(self, logs_loc : str) -> None:
        """Loads docstring into a pandas data frame for efficient searching

        Args:
            logs_loc (str): Location of roslogs aquired during the run
        """
        log_lines = []

        with open(logs_loc, 'r') as file:
            for line in file:
                # You can process the line here if needed
                # strip() removes any leading/trailing whitespace including newlines
                log_lines.append(line.strip())  

        fmt = "[{rospy_node_type}][{log_severity}] {date_time}: {node_log}"

        fmt_info_custom    = "[{rospy_node_type}][{log_severity}] {date_time}: CommandServerLog([{node_name}]): {node_log}"
        fmt_info    = "[{rospy_node_type}][{log_severity}] {date_time}: {node_name}"
        fmt_error   = "[{rospy_node_type}][{log_severity}] {date_time}: {node_name} {node_log}"
        fmt_warning = "[{rospy_node_type}][{log_severity}] {date_time}: {node_name}"
        fmt_debug = "[{rospy_node_type}][{log_severity}] {date_time}: {node_name}"
        fmt_fatal = "[{rospy_node_type}][{log_severity}] {date_time}: {node_name}"
        fmt_list = [fmt_info, fmt_error, fmt_warning, fmt_debug, fmt_fatal]
        parsed_data = []
        log_severity_enum = {'INFO': 0, 'ERROR': 1, 'WARNING': 2, 'DEBUG': 3, 'FATAL': 4}

        
        for i in log_lines:
            if parse(fmt, i) is not None:
                temp_parse = parse(fmt, i).named
            else:
                continue
            if temp_parse['log_severity'] == 'INFO' and temp_parse['rospy_node_type'] == 'rosout' and temp_parse['node_log'] == 'Robot Enabled':
                parsed_data.append(temp_parse)
            elif temp_parse['log_severity'] == 'INFO' and temp_parse['rospy_node_type'] == 'rosout':
                parsed_data.append(parse(fmt_info_custom,i).named)
            elif temp_parse['log_severity'] == 'INFO' and temp_parse['rospy_node_type'] == 'rospy.internal':
                parsed_data.append(parse(fmt_error,i).named)
            else:
                parsed_data.append(parse(fmt_list[log_severity_enum[temp_parse['log_severity']]],i).named)

        self.nodes_logs = pd.DataFrame(parsed_data)

    def bot_loop(self):
        """_summary_
        """

        self.message = ""
        self.init_message()
        self.chatbot_prompt()
        save_conversation = ""
        save_conversation += "User response: " + self.message + "\n \n"
        # Reply after conditioning GPT to be a chat agent
        reply = self.conversation.predict(input=self.message)
        save_conversation += "AI response: " + reply + "\n \n"

        print(Fore.GREEN + reply + Style.RESET_ALL)

        textClassifier = trainClassifier()
        textClassifier.loadLabelsAndModel(
            "QuestionClassificationModel.pkl",
            "ClassificationLabels.json"
            )
        self.load_logs(str(Path.home()) + "/.ros/log/CommandServer.log")

        # Predict is string of an int because text classifier
        # mapping takes str of respective numbers
        # to identify clustered nodes from the KMeans model
        predict = "-1"

        while True:
            if predict != "-1":
                occurance_log = ""
                occourances = self.nodes_logs.node_name.index[self.nodes_logs.node_name == textClassifier.mapping[str(predict[0])]]
                for i in occourances:
                    occurance_log += self.nodes_logs.node_log[i] + " at time: " + self.nodes_logs.date_time[i] + "; "

            self.message = "" if predict == "-1" else f"The log for node {textClassifier.mapping[predict]} is {occurance_log} and the description for the same is {self.ServerFunctions_dict[textClassifier.mapping[predict]].short_description}, I hope this answers the query about the previous doubt, the user may not be aware exactly but this is what the logs say. User's response about real world observation: "
            self.message += input("Enter Your Response: ")

            save_conversation += "User response: " + self.message[21:] + "\n \n"

            reply = self.conversation.predict(input=self.message)

            save_conversation += "AI response: " + reply + "\n \n"

            predict = str(self.cluster_prompt(reply, textClassifier)[0])
            print(Fore.GREEN + reply + Style.RESET_ALL)

            if "complete" in reply and "report" in reply:

                shutil.move(str(Path.home()) + "/.ros/log/CommandServer.log",
                            str(Path.home()) + "/HRIPapers/Experiments/CommandServer.log"
                            )

                with open("Conversation.txt", "w") as text_file:
                    text_file.write(save_conversation)

                shutil.move("Conversation.txt",
                            str(Path.home()) + "/HRIPapers/Experiments/Conversation.txt"
                            )

def bot_test():
    """_summary_
    """
    tt = Bot()
    tt.create_data("../../pick_and_place/src/BTClient.cpp",
                   "../../pick_and_place/scripts/BTNodeServer.py",
                   "../../pick_and_place/src/temp.xml")
    tt.bot_loop()

def training_classifier():
    """_summary_
    """
    tt = trainClassifier()
    # tt.createData("../../pick_and_place/src/temp.xml")
    # tt.loadData("../../pick_and_place/src/temp.xml")
    tt.loadLabelsAndModel("QuestionClassificationModel.pkl", "ClassificationLabels.json")

bot_test()

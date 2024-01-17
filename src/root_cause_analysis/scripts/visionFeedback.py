import openai, os, re
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import pickle, torch, mmcv
import torch.nn.functional as F
import json
import shutil, sys
import numpy as np
import docstring_parser
import dinov2.eval.segmentation.models
import dinov2.eval.segmentation.utils.colormaps as colormaps
import dinov2.eval.segmentation_m2f.models.segmentors

from langchain.llms import OpenAI
from langchain.chat_models import ChatOpenAI
from langchain.memory import ConversationBufferMemory
from langchain.chains import ConversationChain

from sklearn.cluster import KMeans
from sklearn.manifold import TSNE
from statistics import mode
from queue import Queue
from lxml import etree
from parse import parse
from abc import ABC
from comment_parser import comment_parser
from bs4 import BeautifulSoup
from pathlib import Path
from mmseg.apis import init_segmentor, inference_segmentor
from mmcv.runner import load_checkpoint


# https://github.com/facebookresearch/dinov2/blob/main/notebooks/semantic_segmentation.ipynb
INSTALL = False # Switch this to install dependencies
if INSTALL: # Try installing package with extras
    REPO_URL = "https://github.com/facebookresearch/dinov2"
    !{sys.executable} -m pip install -e {REPO_URL}'[extras]' --extra-index-url https://download.pytorch.org/whl/cu117  --extra-index-url https://pypi.nvidia.com
else:
    REPO_PATH = "<FIXME>" # Specify a local path to the repository (or use installed package instead)
    sys.path.append(REPO_PATH)


class visionBot(ABC):
    def __init__(self) -> None:
        super().__init__()
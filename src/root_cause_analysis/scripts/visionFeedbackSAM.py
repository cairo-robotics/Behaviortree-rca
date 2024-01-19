import openai, os, re
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import pickle, torch, mmcv
import torch.nn.functional as F
import json, hydra
import shutil, sys
import numpy as np
import docstring_parser

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
from hydra.core.hydra_config import HydraConfig
from hydra.utils import instantiate
from omegaconf import OmegaConf


class visionBot(ABC):
    def __init__(self) -> None:
        super().__init__()

    def load_model(cfg, positive_points_per_mask, negative_points_per_mask):
        cfg.model.positive_points_per_mask = positive_points_per_mask
        cfg.model.negative_points_per_mask = negative_points_per_mask
        model = instantiate(cfg.model)
        return model.to("cuda" if torch.cuda.is_available() else "cpu").eval()

    def run_inference(model, rgbs, query_points, target_hw):
        n_masks, n_points_per_mask, _ = query_points.shape
        n_frames, _, _, _ = rgbs.shape

        # Prepare inputs
        video = {
            "video_id": 0,
            "image": [rgb for rgb in rgbs],
            "target_hw": target_hw,
            "query_points": query_points,
        }
        device = model.device
        for k, v in video.items():
            if isinstance(v, torch.Tensor):
                video[k] = v.to(device)

        # Forward pass
        outputs = model(video)

        # Unpack outputs
        logits_list = outputs['logits']
        trajectories = outputs['trajectories']
        visibilities = outputs['visibilities']
        scores = outputs['scores']

        # Post-process outputs
        logits = torch.stack([torch.zeros_like(logits_list[0])] + logits_list, dim=1)
        assert torch.all(logits[:, 0] == 0), "We always set the background mask to zero logits"

        assert logits.shape[0] == n_frames
        if trajectories is None:
            trajectories = torch.zeros((n_frames, n_masks, n_points_per_mask, 2), dtype=torch.float32)
            visibilities = torch.zeros((n_frames, n_masks, n_points_per_mask), dtype=torch.float32)
            scores = torch.zeros(n_masks, dtype=torch.float32)
        assert trajectories.shape == (n_frames, n_masks, n_points_per_mask, 2)

        # Post process the predictions to set masks to zero for all frames before the query frame
        for i, timestep in enumerate(query_points[:, 0, 0].tolist()):
            timestep = int(timestep)
            logits[:timestep, i + 1] = -1e8

        return logits, trajectories, visibilities, scores

    def load_query_points(query_points_path, frame_stride, resize_factor):
        query_point_timestep_list = []
        query_points_list = []
        with open(query_points_path, 'r') as f:
            lines = f.readlines()
            num_positive_points = int(lines[0].strip())
            for line in lines[1:]:
                line = line.strip()
                if not line:
                    continue
                timestep, queries_xy = line.split(';')
                queries_xy = [x.split(',') for x in queries_xy.split()]
                queries_xy = [[float(x), float(y)] for x, y in queries_xy]
                queries_xy = torch.tensor(queries_xy)
                queries_xy = queries_xy * resize_factor

                timestep = int(timestep)
                assert timestep % frame_stride == 0
                timestep = timestep // frame_stride

                query_point_timestep_list += [timestep]
                query_points_list += [queries_xy]

        query_points_xy = torch.stack(query_points_list)
        query_points_timestep = torch.tensor(query_point_timestep_list, dtype=torch.float32)[:, None, None]
        query_points = torch.cat([query_points_timestep.repeat(1, query_points_xy.shape[1], 1), query_points_xy], dim=2)

    def load_demo_data(frames_path, query_points_path, frame_stride=1, longest_side_length=None,
                    annot_size=8, annot_line_width=4, max_frames=None):
        assert query_points_path is not None

        # Load frames
        frames = sorted(glob.glob(os.path.join(frames_path, '*.jpg')))
        frames += sorted(glob.glob(os.path.join(frames_path, '*.png')))
        assert len(frames) > 0, f"No frames found in {frames_path}"

        frames = frames[::frame_stride]
        if max_frames is not None:
            frames = frames[:max_frames]

        rgbs = []
        for frame in frames:
            img = cv2.imread(frame)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img = torch.from_numpy(img).permute(2, 0, 1)

            if longest_side_length is not None:
                resize_factor = longest_side_length / max(img.shape[1], img.shape[2])
                img = torch.nn.functional.interpolate(img[None], scale_factor=resize_factor)[0]
            else:
                resize_factor = 1.0

            rgbs += [img]
        rgbs = torch.stack(rgbs)

        # Load query points from a file or select them interactively from the first frame
        query_points, num_positive_points = load_query_points(query_points_path, frame_stride, resize_factor)

        return rgbs, num_positive_points, query_points

        return query_points, num_positive_points

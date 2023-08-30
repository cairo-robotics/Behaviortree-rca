import numpy as np 
from scipy.stats import norm
import matplotlib.pyplot as plt
import json
import yaml

class read_log():
    def __init__(self) -> None:
        pass

class read_settings():
    def __init__(self) -> None:
        pass

class pdf:
    def __init__(self) -> None:
        pass
    def sample(self):
        pass
    def update(self):
        pass


class ParticleFilter:
    def __init__(self) -> None:
        pass

    def generateParticles(self):
        self.particles = np.random.normal(0.5, self.spread, (1,self.N))
        self.weights   = np.random.uniform(0., 1., (1, self.N))

    def predictionModel(self, node, list):
        for i in np.shape(self.particles)[1]:
            pass # Particles update rule based on the list and beliefs of other nodes

    def updateWeights(self):
        # Assuming perfect observation model as we'd know if there is a fault or not 
        weight_dist = norm(0.5, self.spread)
        
        for i in np.shape(self.weights)[1]:
            self.weights = weight_dist.pdf(self.particles[i])/weight_dist.pdf(0.5)

    def filterWeights(self):
        
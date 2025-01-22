import inspect
import logging
import cv2
import torch
import importlib
import numpy as np
from matplotlib.testing import setup
from torch import Tensor
from torch.nn import functional as F
import torch.optim.lr_scheduler as lrs
import pytorch_lightning as pl
from pytorch_lightning.utilities.types import STEP_OUTPUT
import utils
# import models
from models.SuperPoint import SuperPointNet
from models.D2_Net import D2Net
from models.ALike import ALNet
from models.XFeat import XFeatModel
# import tasks
from tasks.repeatability import repeatability, plot_repeatability

import tensorrt as trt
import openvino as ov
def export_model(model, name):
        device = torch.device("cpu")
        model.to(device)
        dummy_input = torch.randn(1, 3, 512, 512, requires_grad=True).to(device)
        torch.onnx.export(model,
                      dummy_input,
                      name+".onnx",
                      export_params=True,
                      opset_version=11,
                      do_constant_folding=True,
                      input_names=['input'],
                      output_names=['score', 'descriptor'])
        ov_model = ov.convert_model(name+".onnx", example_input=dummy_input)
        ov.save_model(ov_model, name+'.xml')

        # export to tensorRT
        TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
        builder = trt.Builder(TRT_LOGGER)
        network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
        parser = trt.OnnxParser(network, TRT_LOGGER)

        with open(name+".onnx", 'rb') as model_file:
            if not parser.parse(model_file.read()):
                for error in range(parser.num_errors):
                    print(parser.get_error(error))
                raise Exception("Failed to parse the ONNX file")
        

        config = builder.create_builder_config()
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)  # 1GB
        serialized_engine = builder.build_serialized_network(network, config)

        runtime = trt.Runtime(TRT_LOGGER)
        engine = runtime.deserialize_cuda_engine(serialized_engine)

        trt_engine_path =  name + ".trt"
        with open(trt_engine_path, "wb") as f:
            f.write(engine.serialize())

class MInterface(pl.LightningModule):
    def __init__(self, params) -> None:
        super().__init__()
        self.params = params
        self.matcher = None
        # model choice
        if params['model_type'] == 'SuperPoint':
            self.model = SuperPointNet()
            self.model.load_state_dict(torch.load(params['SuperPoint_params']['weight']))
        elif params['model_type'] == 'Alike':
            self.model = ALNet(params['Alike_params'])
            self.model.load_state_dict(torch.load(params['Alike_params']['weight']))
        elif params['model_type'] == 'D2Net':
            self.model = D2Net(model_file=params['D2Net_params']['weight'])
        elif params['model_type'] == 'XFeat':
            self.model = XFeatModel()
            self.model.load_state_dict(torch.load(params['XFeat_params']['weight']))
        else:
            raise NotImplementedError
        self.model.eval()
        export_model(self.model,"SuperPoint")
        self.num_feat = None
        self.repeatability = None
        self.rep_mean_err = None
        self.accuracy = None
        self.matching_score = None
        self.track_error = None
        self.last_batch = None
        self.fundamental_error = None
        self.fundamental_radio = None
        self.fundamental_num = None
        self.r_est = None
        self.t_est = None

    def on_test_start(self) -> None:
        self.num_feat = []
        self.repeatability = []
        self.rep_mean_err = []
        self.accuracy = []
        self.matching_score = []
        self.track_error = []
        self.fundamental_error = []
        self.fundamental_radio = []
        self.fundamental_num = []
        self.r_est = [np.eye(3)]
        self.t_est = [np.zeros([3, 1])]

    def on_test_end(self) -> None:
        self.num_feat = np.mean(self.num_feat)
        self.accuracy = np.mean(self.accuracy)
        self.matching_score = np.mean(self.matching_score)
        print('task: ', self.params['task_type'])
        if self.params['task_type'] == 'repeatability':
            rep = torch.as_tensor(self.repeatability).cpu().numpy()
            plot_repeatability(rep, self.params['repeatability_params']['save_path'])
            rep = np.mean(rep)

            ms = torch.as_tensor(self.matching_score).cpu().numpy()
            ms = np.mean(ms)

            error = torch.as_tensor(self.rep_mean_err).cpu().numpy()
            error = error[~np.isnan(error)]
            plot_repeatability(error, self.params['repeatability_params']['save_path'].replace('.png', '_error.png'))
            error = np.mean(error)

            print('repeatability', rep, ' rep_mean_err', error, ' matching_score', ms)

    def test_step(self, batch: Tensor, batch_idx: int) -> STEP_OUTPUT:

        warp01_params = {}
        warp10_params = {}
        if 'warp01_params' in batch:
            for k, v in batch['warp01_params'].items():
                warp01_params[k] = v[0]
            for k, v in batch['warp10_params'].items():
                warp10_params[k] = v[0]

        # pairs dataset
        score_map_0 = None
        score_map_1 = None
        desc_map_0 = None
        desc_map_1 = None

        # image pair dataset
        if batch['dataset'][0] == 'HPatches' or \
           batch['dataset'][0] == 'megaDepth' or \
           batch['dataset'][0] == 'image_pair':
            result0 = self.model(batch['image0'])
            result1 = self.model(batch['image1'])
            score_map_0 = result0[0].detach()
            score_map_1 = result1[0].detach()
            if result0[1] is not None:
                desc_map_0 = result0[1].detach()
                desc_map_1 = result1[1].detach()

        # sequence dataset
        last_img = None

        if batch['dataset'][0] == 'Kitti' or batch['dataset'][0] == 'Euroc' or batch['dataset'][0] == 'TartanAir':
            if self.last_batch is None:
                self.last_batch = batch
            result0 = self.model(self.last_batch['image0'])
            result1 = self.model(batch['image0'])
            score_map_0 = result0[0].detach()
            score_map_1 = result1[0].detach()
            if result0[1] is not None:
                desc_map_0 = result0[1].detach()
                desc_map_1 = result1[1].detach()
            last_img = self.last_batch['image0']
            self.last_batch = batch
        # task
        result = None
        if self.params['task_type'] == 'repeatability':
            result = repeatability(batch_idx, batch['image0'], score_map_0,
                                   batch['image1'], score_map_1,
                                   desc_map_0, desc_map_1,
                                   warp01_params, warp10_params, self.params)
            self.num_feat.append(result['num_feat'])
            self.repeatability.append(result['repeatability'])
            self.rep_mean_err.append(result['mean_error'])
            self.matching_score.append(result['matching_score'])
        return result

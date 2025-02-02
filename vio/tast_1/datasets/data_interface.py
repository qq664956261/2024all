from torch.utils.data import DataLoader
from pytorch_lightning import LightningDataModule
from datasets.hpatches import HPatchesDataset


class DInterface(LightningDataModule):

    def __init__(self, params):
        super().__init__()
        self.num_workers = params['num_workers']
        self.batch_size = params['batch_size']
        data_type = params['data_type']
        self.test_set_param = params[data_type+'_params']

    def setup(self, stage=None):
        # Assign train/val datasets for use in dataloaders
        if stage == 'fit' or stage is None:
            self.trainset = self.instancialize(self.trainset_param, train=True)
            self.valset = self.instancialize(self.valset_param, train=False)

        # Assign test dataset for use in dataloader(s)
        if stage == 'test' or stage is None:
            self.test_set = self.instancialize(self.test_set_param, train=False)

    def train_dataloader(self):
        return DataLoader(self.trainset, batch_size=self.batch_size, num_workers=self.num_workers, shuffle=True)

    def val_dataloader(self):
        return DataLoader(self.valset, batch_size=self.batch_size, num_workers=self.num_workers, shuffle=False)

    def test_dataloader(self):
        return DataLoader(self.test_set, batch_size=self.batch_size, num_workers=self.num_workers, shuffle=False)

    def instancialize(self, params, train=True):
        """ Instancialize a model using the corresponding parameters
            from self.hparams dictionary. You can also input any args
            to overwrite the corresponding value in self.kwargs.
        """
        if params['type'] == 'hpatches':
            return HPatchesDataset(params['root'], params['alteration'], params['image_size'], params['gray'])
        else:
            raise ValueError('Invalid dataset type')


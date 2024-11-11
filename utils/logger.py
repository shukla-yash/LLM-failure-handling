from torch.utils.tensorboard import SummaryWriter
import numpy as np
import time
import json

# slightly modified from dreamer-pytorch, add config logging
class Logger:
    def __init__(self, path, run_name, step):
        self._path = path
        self._run_name = run_name
        self._logdir = path / run_name if str(run_name) != "." else path / "default_log"
        if not self._logdir.exists():
            self._logdir.mkdir(parents=True)
        self._writer = SummaryWriter(self._logdir)  
        self._last_step = None
        self._last_time = None
        self._scalars = {}
        self._images = {}
        self._videos = {}
        self.step = step

    def scalar(self, name, value):
        self._scalars[name] = float(value)

    def image(self, name, value):
        self._images[name] = np.array(value)

    def video(self, name, value):
        self._videos[name] = np.array(value)

    def write(self, fps=False, step=False):
        '''
        Write all the logged scalars, images and videos to tensorboard
        '''
        if not step:
            step = self.step

        scalars = list(self._scalars.items())
        if not fps:
            scalars.append(("fps", self._compute_fps(step)))
        # print all the logged scalars
        print(f"[{step}]", " / ".join(f"{k} {v:.1f}" for k, v in scalars))
        # write scalars to jsonl file
        with (self._logdir / "metrics.jsonl").open("a") as f:
            f.write(json.dumps({"step": step, **dict(scalars)}) + "\n")
        # write scalars to tensorboard
        for name, value in scalars:
            if "/" not in name:
                self._writer.add_scalar("scalars/" + name, value, step)
            else:
                self._writer.add_scalar(name, value, step)
        # write images to tensorboard
        for name, value in self._images.items():
            self._writer.add_image(name, value, step)
        # write videos to tensorboard
        for name, value in self._videos.items():
            name = name if isinstance(name, str) else name.decode("utf-8")
            if np.issubdtype(value.dtype, np.floating):
                print("value dtype", value.dtype)
                value = np.clip(255 * value, 0, 255).astype(np.uint8)
            B, T, H, W, C = value.shape
            # (B, T, H, W, C) -> (1, T, C, H, B * W)
            value = value.transpose(1, 4, 2, 0, 3).reshape((1, T, C, H, B * W))
            self._writer.add_video(name, value, step, 16)

        self._writer.flush()
        self._scalars = {}
        self._images = {}
        self._videos = {}

    def log_config(self, config):
        '''
        Log the configuration parameters to tensorboard
        '''
        with (self._logdir / "config.json").open("w") as f:
            json.dump(config, f, indent=4)


        
    def _compute_fps(self, step):
        '''
        Compute the frames per second for video logging
        '''
        if self._last_step is None:
            self._last_time = time.time()
            self._last_step = step
            return 0
        steps = step - self._last_step
        duration = time.time() - self._last_time
        self._last_time += duration
        self._last_step = step
        return steps / duration
    
    def offline_scalar(self, name, value, step):
        '''
        log a single scalar value to tensorboard
        '''
        self._writer.add_scalar("scalars/" + name, value, step)

    def offline_video(self, name, value, step):
        '''
        log a single video to tensorboard
        '''
        if np.issubdtype(value.dtype, np.floating):
            value = np.clip(255 * value, 0, 255).astype(np.uint8)
        B, T, H, W, C = value.shape
        value = value.transpose(1, 4, 2, 0, 3).reshape((1, T, C, H, B * W))
        self._writer.add_video(name, value, step, 16)

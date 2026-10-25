"""Reuse the user's inspected NCNN preprocessing and tracker; exclude distance guesses."""
import importlib.util
from pathlib import Path
import yaml


class ModelAdapter:
    def __init__(self, deployment_dir, threads=2, confidence=0.35):
        root = Path(deployment_dir).resolve()
        metadata = yaml.safe_load((root / 'model/metadata.yaml').read_text())
        if metadata['task'] != 'detect' or list(metadata['names'].values()) != ['person']:
            raise ValueError('Expected the supplied single-class person detection model')
        if metadata['imgsz'] != [512, 512]:
            raise ValueError('This adapter is validated for the supplied 512x512 export')
        if threads not in (1, 2) or not 0 < confidence <= 1:
            raise ValueError('Use 1 or 2 inference threads and confidence in (0,1]')
        source = root / 'src/hybrid_tracker.py'
        spec = importlib.util.spec_from_file_location('as2_existing_person_tracker', source)
        module = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(module)
        except SystemExit as exc:
            raise RuntimeError('Existing tracker dependencies could not be imported') from exc
        module.cv2.setNumThreads(1)
        module.CONF_THRESH = confidence
        self.module = module
        self.net = module.ncnn.Net()
        self.net.opt.num_threads = threads
        for method, filename in [(self.net.load_param, 'model.ncnn.param'),
                                 (self.net.load_model, 'model.ncnn.bin')]:
            path = root / 'model' / filename
            if not path.is_file() or method(str(path)) != 0:
                raise RuntimeError(f'NCNN could not load {path}')
        self.tracker = module.HybridKalmanTracker(max_age=5, min_hits=2)

    def infer(self, frame):
        # Existing method uses this instance's validated net and existing preprocessing.
        return self.module.NCNNInferencer.infer(self, frame)

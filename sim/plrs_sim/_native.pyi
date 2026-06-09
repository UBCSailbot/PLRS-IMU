"""Type stubs for the nanobind-compiled FFI surface."""

class Vec3:
    x: float
    y: float
    z: float
    def __init__(self, x: float = ..., y: float = ..., z: float = ...) -> None: ...

class Quaternion:
    w: float
    x: float
    y: float
    z: float
    def __init__(
        self, w: float = ..., x: float = ..., y: float = ..., z: float = ...
    ) -> None: ...

class UnitQuaternion:
    @staticmethod
    def from_raw(q: Quaternion) -> UnitQuaternion: ...
    @staticmethod
    def identity() -> UnitQuaternion: ...

class MountRotation:
    boat_to_imu: UnitQuaternion
    def __init__(self) -> None: ...

class ImuSample:
    angular_velocity_rad_s: Vec3
    accel_ms2: Vec3
    orientation: UnitQuaternion
    timestamp_ms: int
    def __init__(self) -> None: ...

class GnssSample:
    heading_deg: float
    heading_variance_deg2: float
    valid: bool
    timestamp_ms: int
    def __init__(self) -> None: ...

class FusionOutput:
    heading_deg: float
    heading_variance_deg2: float
    roll_deg: float
    roll_variance_deg2: float
    pitch_deg: float
    pitch_variance_deg2: float
    timestamp_ms: int

class Config:
    q_heading_deg2: float
    q_roll_deg2: float
    q_pitch_deg2: float
    q_bias_deg2_s2: float
    p0_heading_deg2: float
    p0_roll_deg2: float
    p0_pitch_deg2: float
    p0_bias_deg2_s2: float
    mti_roll_variance_deg2: float
    mti_pitch_variance_deg2: float
    mount: MountRotation
    def __init__(self) -> None: ...

class TinyEkfFilter:
    def __init__(self, cfg: Config) -> None: ...
    def predict(self, imu: ImuSample) -> None: ...
    def update(self, gnss: GnssSample) -> None: ...
    def output(self) -> FusionOutput: ...

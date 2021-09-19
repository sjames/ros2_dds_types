use cyclonedds_rs::*;
use dds_derive::Topic;
use serde_big_array::*;
use serde_derive::{Deserialize, Serialize};

#[repr(C)]
#[derive(Deserialize, Serialize, Topic)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}

#[repr(C)]
#[derive(Deserialize, Serialize, Topic)]
pub struct Duration {
    pub sec: i32,
    pub nanosec: u32,
}

pub mod sensor_msgs {
    use super::*;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Image {
        pub header: std_msgs::Header,
        pub height: u32,
        pub width: u32,
        pub encoding: std::string::String,
        pub is_bigendian: u8,
        pub step: u32,
        pub data: Vec<u8>,
    }
    const BatteryState_POWER_SUPPLY_STATUS_UNKNOWN: u8 = 0;
    const BatteryState_POWER_SUPPLY_STATUS_CHARGING: u8 = 1;
    const BatteryState_POWER_SUPPLY_STATUS_DISCHARGING: u8 = 2;
    const BatteryState_POWER_SUPPLY_STATUS_NOT_CHARGING: u8 = 3;
    const BatteryState_POWER_SUPPLY_STATUS_FULL: u8 = 4;
    const BatteryState_POWER_SUPPLY_HEALTH_UNKNOWN: u8 = 0;
    const BatteryState_POWER_SUPPLY_HEALTH_GOOD: u8 = 1;
    const BatteryState_POWER_SUPPLY_HEALTH_OVERHEAT: u8 = 2;
    const BatteryState_POWER_SUPPLY_HEALTH_DEAD: u8 = 3;
    const BatteryState_POWER_SUPPLY_HEALTH_OVERVOLTAGE: u8 = 4;
    const BatteryState_POWER_SUPPLY_HEALTH_UNSPEC_FAILURE: u8 = 5;
    const BatteryState_POWER_SUPPLY_HEALTH_COLD: u8 = 6;
    const BatteryState_POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE: u8 = 7;
    const BatteryState_POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE: u8 = 8;
    const BatteryState_POWER_SUPPLY_TECHNOLOGY_UNKNOWN: u8 = 0;
    const BatteryState_POWER_SUPPLY_TECHNOLOGY_NIMH: u8 = 1;
    const BatteryState_POWER_SUPPLY_TECHNOLOGY_LION: u8 = 2;
    const BatteryState_POWER_SUPPLY_TECHNOLOGY_LIPO: u8 = 3;
    const BatteryState_POWER_SUPPLY_TECHNOLOGY_LIFE: u8 = 4;
    const BatteryState_POWER_SUPPLY_TECHNOLOGY_NICD: u8 = 5;
    const BatteryState_POWER_SUPPLY_TECHNOLOGY_LIMN: u8 = 6;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct BatteryState {
        pub header: std_msgs::Header,
        pub voltage: f32,
        pub temperature: f32,
        pub current: f32,
        pub charge: f32,
        pub capacity: f32,
        pub design_capacity: f32,
        pub percentage: f32,
        pub power_supply_status: u8,
        pub power_supply_health: u8,
        pub power_supply_technology: u8,
        pub present: bool,
        pub cell_voltage: Vec<f32>,
        pub cell_temperature: Vec<f32>,
        pub location: std::string::String,
        pub serial_number: std::string::String,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Imu {
        pub header: std_msgs::Header,
        pub orientation: geometry_msgs::Quaternion,
        pub orientation_covariance: [f64; 9],
        pub angular_velocity: geometry_msgs::Vector3,
        pub angular_velocity_covariance: [f64; 9],
        pub linear_acceleration: geometry_msgs::Vector3,
        pub linear_acceleration_covariance: [f64; 9],
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Illuminance {
        pub header: std_msgs::Header,
        pub illuminance: f64,
        pub variance: f64,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct TimeReference {
        pub header: std_msgs::Header,
        pub time_ref: Time,
        pub source: std::string::String,
    }
    const NavSatFix_COVARIANCE_TYPE_UNKNOWN: u8 = 0;
    const NavSatFix_COVARIANCE_TYPE_APPROXIMATED: u8 = 1;
    const NavSatFix_COVARIANCE_TYPE_DIAGONAL_KNOWN: u8 = 2;
    const NavSatFix_COVARIANCE_TYPE_KNOWN: u8 = 3;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct NavSatFix {
        pub header: std_msgs::Header,
        pub status: NavSatStatus,
        pub latitude: f64,
        pub longitude: f64,
        pub altitude: f64,
        pub position_covariance: [f64; 9],
        pub position_covariance_type: u8,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct FluidPressure {
        pub header: std_msgs::Header,
        pub fluid_pressure: f64,
        pub variance: f64,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct ChannelFloat32 {
        pub name: std::string::String,
        pub values: Vec<f32>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct LaserEcho {
        pub echoes: Vec<f32>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct JoyFeedbackArray {
        pub array: Vec<JoyFeedback>,
    }
    const JoyFeedback_TYPE_LED: u8 = 0;
    const JoyFeedback_TYPE_RUMBLE: u8 = 1;
    const JoyFeedback_TYPE_BUZZER: u8 = 2;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct JoyFeedback {
        pub r#type: u8,
        pub id: u8,
        pub intensity: f32,
    }
    const PointField_INT8: u8 = 1;
    const PointField_UINT8: u8 = 2;
    const PointField_INT16: u8 = 3;
    const PointField_UINT16: u8 = 4;
    const PointField_INT32: u8 = 5;
    const PointField_UINT32: u8 = 6;
    const PointField_FLOAT32: u8 = 7;
    const PointField_FLOAT64: u8 = 8;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct PointField {
        pub name: std::string::String,
        pub offset: u32,
        pub datatype: u8,
        pub count: u32,
    }
    const NavSatStatus_STATUS_NO_FIX: i8 = -1;
    const NavSatStatus_STATUS_FIX: i8 = 0;
    const NavSatStatus_STATUS_SBAS_FIX: i8 = 1;
    const NavSatStatus_STATUS_GBAS_FIX: i8 = 2;
    const NavSatStatus_SERVICE_GPS: u16 = 1;
    const NavSatStatus_SERVICE_GLONASS: u16 = 2;
    const NavSatStatus_SERVICE_COMPASS: u16 = 4;
    const NavSatStatus_SERVICE_GALILEO: u16 = 8;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct NavSatStatus {
        pub status: i8,
        pub service: u16,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct CameraInfo {
        pub header: std_msgs::Header,
        pub height: u32,
        pub width: u32,
        pub distortion_model: std::string::String,
        pub d: Vec<f64>,
        pub k: [f64; 9],
        pub r: [f64; 9],
        pub p: [f64; 12],
        pub binning_x: u32,
        pub binning_y: u32,
        pub roi: RegionOfInterest,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct LaserScan {
        pub header: std_msgs::Header,
        pub angle_min: f32,
        pub angle_max: f32,
        pub angle_increment: f32,
        pub time_increment: f32,
        pub scan_time: f32,
        pub range_min: f32,
        pub range_max: f32,
        pub ranges: Vec<f32>,
        pub intensities: Vec<f32>,
    }
    const Range_ULTRASOUND: u8 = 0;
    const Range_INFRARED: u8 = 1;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Range {
        pub header: std_msgs::Header,
        pub radiation_type: u8,
        pub field_of_view: f32,
        pub min_range: f32,
        pub max_range: f32,
        pub range: f32,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct PointCloud2 {
        pub header: std_msgs::Header,
        pub height: u32,
        pub width: u32,
        pub fields: Vec<PointField>,
        pub is_bigendian: bool,
        pub point_step: u32,
        pub row_step: u32,
        pub data: Vec<u8>,
        pub is_dense: bool,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct MagneticField {
        pub header: std_msgs::Header,
        pub magnetic_field: geometry_msgs::Vector3,
        pub magnetic_field_covariance: [f64; 9],
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct MultiDOFJointState {
        pub header: std_msgs::Header,
        pub joint_names: Vec<std::string::String>,
        pub transforms: Vec<geometry_msgs::Transform>,
        pub twist: Vec<geometry_msgs::Twist>,
        pub wrench: Vec<geometry_msgs::Wrench>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Joy {
        pub header: std_msgs::Header,
        pub axes: Vec<f32>,
        pub buttons: Vec<i32>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct MultiEchoLaserScan {
        pub header: std_msgs::Header,
        pub angle_min: f32,
        pub angle_max: f32,
        pub angle_increment: f32,
        pub time_increment: f32,
        pub scan_time: f32,
        pub range_min: f32,
        pub range_max: f32,
        pub ranges: Vec<LaserEcho>,
        pub intensities: Vec<LaserEcho>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct RegionOfInterest {
        pub x_offset: u32,
        pub y_offset: u32,
        pub height: u32,
        pub width: u32,
        pub do_rectify: bool,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct RelativeHumidity {
        pub header: std_msgs::Header,
        pub relative_humidity: f64,
        pub variance: f64,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct CompressedImage {
        pub header: std_msgs::Header,
        pub format: std::string::String,
        pub data: Vec<u8>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct JointState {
        pub header: std_msgs::Header,
        pub name: Vec<std::string::String>,
        pub position: Vec<f64>,
        pub velocity: Vec<f64>,
        pub effort: Vec<f64>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Temperature {
        pub header: std_msgs::Header,
        pub temperature: f64,
        pub variance: f64,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct PointCloud {
        pub header: std_msgs::Header,
        pub points: Vec<geometry_msgs::Point32>,
        pub channels: Vec<ChannelFloat32>,
    }
}
pub mod trajectory_msgs {
    use super::*;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct MultiDOFJointTrajectory {
        pub header: std_msgs::Header,
        pub joint_names: Vec<std::string::String>,
        pub points: Vec<MultiDOFJointTrajectoryPoint>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct MultiDOFJointTrajectoryPoint {
        pub transforms: Vec<geometry_msgs::Transform>,
        pub velocities: Vec<geometry_msgs::Twist>,
        pub accelerations: Vec<geometry_msgs::Twist>,
        pub time_from_start: Duration,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct JointTrajectoryPoint {
        pub positions: Vec<f64>,
        pub velocities: Vec<f64>,
        pub accelerations: Vec<f64>,
        pub effort: Vec<f64>,
        pub time_from_start: Duration,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct JointTrajectory {
        pub header: std_msgs::Header,
        pub joint_names: Vec<std::string::String>,
        pub points: Vec<JointTrajectoryPoint>,
    }
}
pub mod diagnostic_msgs {
    use super::*;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct DiagnosticArray {
        pub header: std_msgs::Header,
        pub status: Vec<DiagnosticStatus>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct KeyValue {
        pub key: std::string::String,
        pub value: std::string::String,
    }
    const DiagnosticStatus_OK: u8 = 0;
    const DiagnosticStatus_WARN: u8 = 1;
    const DiagnosticStatus_ERROR: u8 = 2;
    const DiagnosticStatus_STALE: u8 = 3;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct DiagnosticStatus {
        pub level: u8,
        pub name: std::string::String,
        pub message: std::string::String,
        pub hardware_id: std::string::String,
        pub values: Vec<KeyValue>,
    }
}
pub mod shape_msgs {
    use super::*;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct MeshTriangle {
        pub vertex_indices: [u32; 3],
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Mesh {
        pub triangles: Vec<MeshTriangle>,
        pub vertices: Vec<geometry_msgs::Point>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Plane {
        pub coef: [f64; 4],
    }
    const SolidPrimitive_BOX: u8 = 1;
    const SolidPrimitive_SPHERE: u8 = 2;
    const SolidPrimitive_CYLINDER: u8 = 3;
    const SolidPrimitive_CONE: u8 = 4;
    const SolidPrimitive_BOX_X: u8 = 0;
    const SolidPrimitive_BOX_Y: u8 = 1;
    const SolidPrimitive_BOX_Z: u8 = 2;
    const SolidPrimitive_SPHERE_RADIUS: u8 = 0;
    const SolidPrimitive_CYLINDER_HEIGHT: u8 = 0;
    const SolidPrimitive_CYLINDER_RADIUS: u8 = 1;
    const SolidPrimitive_CONE_HEIGHT: u8 = 0;
    const SolidPrimitive_CONE_RADIUS: u8 = 1;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct SolidPrimitive {
        pub r#type: u8,
    }
}
pub mod visualization_msgs {
    use super::*;
    const Marker_ARROW: i32 = 0;
    const Marker_CUBE: i32 = 1;
    const Marker_SPHERE: i32 = 2;
    const Marker_CYLINDER: i32 = 3;
    const Marker_LINE_STRIP: i32 = 4;
    const Marker_LINE_LIST: i32 = 5;
    const Marker_CUBE_LIST: i32 = 6;
    const Marker_SPHERE_LIST: i32 = 7;
    const Marker_POINTS: i32 = 8;
    const Marker_TEXT_VIEW_FACING: i32 = 9;
    const Marker_MESH_RESOURCE: i32 = 10;
    const Marker_TRIANGLE_LIST: i32 = 11;
    const Marker_ADD: i32 = 0;
    const Marker_MODIFY: i32 = 0;
    const Marker_DELETE: i32 = 2;
    const Marker_DELETEALL: i32 = 3;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Marker {
        pub header: std_msgs::Header,
        pub ns: std::string::String,
        pub id: i32,
        pub r#type: i32,
        pub action: i32,
        pub pose: geometry_msgs::Pose,
        pub scale: geometry_msgs::Vector3,
        pub color: std_msgs::ColorRGBA,
        pub lifetime: Duration,
        pub frame_locked: bool,
        pub points: Vec<geometry_msgs::Point>,
        pub colors: Vec<std_msgs::ColorRGBA>,
        pub texture_resource: std::string::String,
        pub texture: sensor_msgs::CompressedImage,
        pub uv_coordinates: Vec<UVCoordinate>,
        pub text: std::string::String,
        pub mesh_resource: std::string::String,
        pub mesh_file: MeshFile,
        pub mesh_use_embedded_materials: bool,
    }
    const InteractiveMarkerFeedback_KEEP_ALIVE: u8 = 0;
    const InteractiveMarkerFeedback_POSE_UPDATE: u8 = 1;
    const InteractiveMarkerFeedback_MENU_SELECT: u8 = 2;
    const InteractiveMarkerFeedback_BUTTON_CLICK: u8 = 3;
    const InteractiveMarkerFeedback_MOUSE_DOWN: u8 = 4;
    const InteractiveMarkerFeedback_MOUSE_UP: u8 = 5;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct InteractiveMarkerFeedback {
        pub header: std_msgs::Header,
        pub client_id: std::string::String,
        pub marker_name: std::string::String,
        pub control_name: std::string::String,
        pub event_type: u8,
        pub pose: geometry_msgs::Pose,
        pub menu_entry_id: u32,
        pub mouse_point: geometry_msgs::Point,
        pub mouse_point_valid: bool,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct MeshFile {
        pub filename: std::string::String,
        pub data: Vec<u8>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct MarkerArray {
        pub markers: Vec<Marker>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct InteractiveMarker {
        pub header: std_msgs::Header,
        pub pose: geometry_msgs::Pose,
        pub name: std::string::String,
        pub description: std::string::String,
        pub scale: f32,
        pub menu_entries: Vec<MenuEntry>,
        pub controls: Vec<InteractiveMarkerControl>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct UVCoordinate {
        pub u: f32,
        pub v: f32,
    }
    const InteractiveMarkerControl_INHERIT: u8 = 0;
    const InteractiveMarkerControl_FIXED: u8 = 1;
    const InteractiveMarkerControl_VIEW_FACING: u8 = 2;
    const InteractiveMarkerControl_NONE: u8 = 0;
    const InteractiveMarkerControl_MENU: u8 = 1;
    const InteractiveMarkerControl_BUTTON: u8 = 2;
    const InteractiveMarkerControl_MOVE_AXIS: u8 = 3;
    const InteractiveMarkerControl_MOVE_PLANE: u8 = 4;
    const InteractiveMarkerControl_ROTATE_AXIS: u8 = 5;
    const InteractiveMarkerControl_MOVE_ROTATE: u8 = 6;
    const InteractiveMarkerControl_MOVE_3D: u8 = 7;
    const InteractiveMarkerControl_ROTATE_3D: u8 = 8;
    const InteractiveMarkerControl_MOVE_ROTATE_3D: u8 = 9;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct InteractiveMarkerControl {
        pub name: std::string::String,
        pub orientation: geometry_msgs::Quaternion,
        pub orientation_mode: u8,
        pub interaction_mode: u8,
        pub always_visible: bool,
        pub markers: Vec<Marker>,
        pub independent_marker_orientation: bool,
        pub description: std::string::String,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct InteractiveMarkerInit {
        pub server_id: std::string::String,
        pub seq_num: u64,
        pub markers: Vec<InteractiveMarker>,
    }
    const InteractiveMarkerUpdate_KEEP_ALIVE: u8 = 0;
    const InteractiveMarkerUpdate_UPDATE: u8 = 1;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct InteractiveMarkerUpdate {
        pub server_id: std::string::String,
        pub seq_num: u64,
        pub r#type: u8,
        pub markers: Vec<InteractiveMarker>,
        pub poses: Vec<InteractiveMarkerPose>,
        pub erases: Vec<std::string::String>,
    }
    const ImageMarker_CIRCLE: i32 = 0;
    const ImageMarker_LINE_STRIP: i32 = 1;
    const ImageMarker_LINE_LIST: i32 = 2;
    const ImageMarker_POLYGON: i32 = 3;
    const ImageMarker_POINTS: i32 = 4;
    const ImageMarker_ADD: i32 = 0;
    const ImageMarker_REMOVE: i32 = 1;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct ImageMarker {
        pub header: std_msgs::Header,
        pub ns: std::string::String,
        pub id: i32,
        pub r#type: i32,
        pub action: i32,
        pub position: geometry_msgs::Point,
        pub scale: f32,
        pub outline_color: std_msgs::ColorRGBA,
        pub filled: u8,
        pub fill_color: std_msgs::ColorRGBA,
        pub lifetime: Duration,
        pub points: Vec<geometry_msgs::Point>,
        pub outline_colors: Vec<std_msgs::ColorRGBA>,
    }
    const MenuEntry_FEEDBACK: u8 = 0;
    const MenuEntry_ROSRUN: u8 = 1;
    const MenuEntry_ROSLAUNCH: u8 = 2;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct MenuEntry {
        pub id: u32,
        pub parent_id: u32,
        pub title: std::string::String,
        pub command: std::string::String,
        pub command_type: u8,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct InteractiveMarkerPose {
        pub header: std_msgs::Header,
        pub pose: geometry_msgs::Pose,
        pub name: std::string::String,
    }
}
pub mod geometry_msgs {
    use super::*;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Point32 {
        pub x: f32,
        pub y: f32,
        pub z: f32,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct WrenchStamped {
        pub header: std_msgs::Header,
        pub wrench: Wrench,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct PoseWithCovariance {
        pub pose: Pose,
        #[serde(with = "BigArray")]
        pub covariance: [f64; 36],
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct PoseWithCovarianceStamped {
        pub header: std_msgs::Header,
        pub pose: PoseWithCovariance,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Transform {
        pub translation: Vector3,
        pub rotation: Quaternion,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct TwistWithCovariance {
        pub twist: Twist,
        #[serde(with = "BigArray")]
        pub covariance: [f64; 36],
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct TwistWithCovarianceStamped {
        pub header: std_msgs::Header,
        pub twist: TwistWithCovariance,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Accel {
        pub linear: Vector3,
        pub angular: Vector3,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct AccelStamped {
        pub header: std_msgs::Header,
        pub accel: Accel,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct AccelWithCovarianceStamped {
        pub header: std_msgs::Header,
        pub accel: AccelWithCovariance,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct PointStamped {
        pub header: std_msgs::Header,
        pub point: Point,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct PoseArray {
        pub header: std_msgs::Header,
        pub poses: Vec<Pose>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Twist {
        pub linear: Vector3,
        pub angular: Vector3,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct TransformStamped {
        pub header: std_msgs::Header,
        pub child_frame_id: std::string::String,
        pub transform: Transform,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct QuaternionStamped {
        pub header: std_msgs::Header,
        pub quaternion: Quaternion,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Pose2D {
        pub x: f64,
        pub y: f64,
        pub theta: f64,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct PoseStamped {
        pub header: std_msgs::Header,
        pub pose: Pose,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Polygon {
        pub points: Vec<Point32>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Point {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Inertia {
        pub m: f64,
        pub com: geometry_msgs::Vector3,
        pub ixx: f64,
        pub ixy: f64,
        pub ixz: f64,
        pub iyy: f64,
        pub iyz: f64,
        pub izz: f64,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct InertiaStamped {
        pub header: std_msgs::Header,
        pub inertia: Inertia,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Pose {
        pub position: Point,
        pub orientation: Quaternion,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Vector3Stamped {
        pub header: std_msgs::Header,
        pub vector: Vector3,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct PolygonStamped {
        pub header: std_msgs::Header,
        pub polygon: Polygon,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Vector3 {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct AccelWithCovariance {
        pub accel: Accel,
        #[serde(with = "BigArray")]
        pub covariance: [f64; 36],
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Wrench {
        pub force: Vector3,
        pub torque: Vector3,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct TwistStamped {
        pub header: std_msgs::Header,
        pub twist: Twist,
    }

    /// orientation in free space in quaternion form.
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Quaternion {
        pub x: f64,
        pub y: f64,
        pub z: f64,
        pub w: f64,
    }

    impl Default for Quaternion {
        fn default() -> Self {
            Self {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            }
        }
    }
}
pub mod stereo_msgs {
    use super::*;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct DisparityImage {
        pub header: std_msgs::Header,
        pub image: sensor_msgs::Image,
        pub f: f32,
        pub t: f32,
        pub valid_window: sensor_msgs::RegionOfInterest,
        pub min_disparity: f32,
        pub max_disparity: f32,
        pub delta_d: f32,
    }
}
pub mod actionlib_msgs {
    use super::*;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct GoalID {
        pub stamp: Time,
        pub id: std::string::String,
    }
    const GoalStatus_PENDING: u8 = 0;
    const GoalStatus_ACTIVE: u8 = 1;
    const GoalStatus_PREEMPTED: u8 = 2;
    const GoalStatus_SUCCEEDED: u8 = 3;
    const GoalStatus_ABORTED: u8 = 4;
    const GoalStatus_REJECTED: u8 = 5;
    const GoalStatus_PREEMPTING: u8 = 6;
    const GoalStatus_RECALLING: u8 = 7;
    const GoalStatus_RECALLED: u8 = 8;
    const GoalStatus_LOST: u8 = 9;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct GoalStatus {
        pub goal_id: GoalID,
        pub status: u8,
        pub text: std::string::String,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct GoalStatusArray {
        pub header: std_msgs::Header,
        pub status_list: Vec<GoalStatus>,
    }
}
pub mod nav_msgs {
    use super::*;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct OccupancyGrid {
        pub header: std_msgs::Header,
        pub info: MapMetaData,
        pub data: Vec<i8>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct GridCells {
        pub header: std_msgs::Header,
        pub cell_width: f32,
        pub cell_height: f32,
        pub cells: Vec<geometry_msgs::Point>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Odometry {
        pub header: std_msgs::Header,
        pub child_frame_id: std::string::String,
        pub pose: geometry_msgs::PoseWithCovariance,
        pub twist: geometry_msgs::TwistWithCovariance,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct MapMetaData {
        pub map_load_time: Time,
        pub resolution: f32,
        pub width: u32,
        pub height: u32,
        pub origin: geometry_msgs::Pose,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Path {
        pub header: std_msgs::Header,
        pub poses: Vec<geometry_msgs::PoseStamped>,
    }
}
pub mod std_msgs {
    use super::*;
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct MultiArrayLayout {
        pub dim: Vec<MultiArrayDimension>,
        pub data_offset: u32,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Int32MultiArray {
        pub layout: MultiArrayLayout,
        pub data: Vec<i32>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Float32 {
        pub data: f32,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Int64MultiArray {
        pub layout: MultiArrayLayout,
        pub data: Vec<i64>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct UInt32 {
        pub data: u32,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Header {
        pub stamp: Time,
        pub frame_id: std::string::String,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Bool {
        pub data: bool,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct ByteMultiArray {
        pub layout: MultiArrayLayout,
        pub data: Vec<u8>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct UInt64MultiArray {
        pub layout: MultiArrayLayout,
        pub data: Vec<u64>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct UInt8MultiArray {
        pub layout: MultiArrayLayout,
        pub data: Vec<u8>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct ColorRGBA {
        pub r: f32,
        pub g: f32,
        pub b: f32,
        pub a: f32,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct UInt16MultiArray {
        pub layout: MultiArrayLayout,
        pub data: Vec<u16>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Int8MultiArray {
        pub layout: MultiArrayLayout,
        pub data: Vec<i8>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct UInt8 {
        pub data: u8,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Byte {
        pub data: u8,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Int8 {
        pub data: i8,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct UInt32MultiArray {
        pub layout: MultiArrayLayout,
        pub data: Vec<u32>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Int16 {
        pub data: i16,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct MultiArrayDimension {
        pub label: std::string::String,
        pub size: u32,
        pub stride: u32,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Int32 {
        pub data: i32,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct UInt64 {
        pub data: u64,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Char {
        pub data: std::os::raw::c_char,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct UInt16 {
        pub data: u16,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Int64 {
        pub data: i64,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Empty {}
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Int16MultiArray {
        pub layout: MultiArrayLayout,
        pub data: Vec<i16>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Float64MultiArray {
        pub layout: MultiArrayLayout,
        pub data: Vec<f64>,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Float64 {
        pub data: f64,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct String {
        pub data: std::string::String,
    }
    #[repr(C)]
    #[derive(Deserialize, Serialize, Topic)]
    pub struct Float32MultiArray {
        pub layout: MultiArrayLayout,
        pub data: Vec<f32>,
    }
}

#include "camera.h"

static const vec3 sCameraUp(0.0f, 1.0f, 0.0f);

Camera::Camera()
    : mFovY(65.0f)
    , mNearZ(1.0f)
    , mFarZ(1000.0f)
    , mPosition(0.0f, 0.0f, 0.0f)
    , mDirection(0.0f, 0.0f, 1.0f)
{
    SyncAnglesWithDirection();
}

Camera::~Camera() {
}

void Camera::SetViewport(const Recti& viewport) {
    mViewport = viewport;
    this->MakeProjection();
}

void Camera::SetFovY(const float fovy) {
    mFovY = fovy;
    this->MakeProjection();
}

void Camera::SetViewPlanes(const float nearZ, const float farZ) {
    mNearZ = nearZ;
    mFarZ = farZ;
    this->MakeProjection();
}

void Camera::SetPosition(const vec3& pos) {
    mPosition = pos;
    this->MakeTransform();
}

void Camera::LookAt(const vec3& pos, const vec3& target) {
    mPosition = pos;
    mDirection = normalize(target - pos);
    SyncAnglesWithDirection();
    this->MakeTransform();
}

void Camera::Move(const float side, const float direction, const float vertical) {
    // Calculate a robust side vector for horizontal movement
    vec3 forward = mDirection;
    vec3 horizontalForward = normalize(vec3(forward.x, 0.0f, forward.z));
    
    // If the camera is looking perfectly up or down, the projection onto XZ is zero.
    // Fallback to the direction vector or a default if looking exactly vertical.
    if (length(horizontalForward) < 0.001f) {
        // Looking straight down: Forward becomes -Z or some consistent direction
        horizontalForward = vec3(0.0f, 0.0f, -1.0f);
    }

    vec3 cameraSide = normalize(cross(horizontalForward, sCameraUp));

    // Horizontal movement in the ground plane (XZ)
    mPosition += cameraSide * side;
    mPosition += horizontalForward * direction;

    // Vertical movement is ALWAYS along global Up (Y)
    mPosition += sCameraUp * vertical;

    this->MakeTransform();
}

void Camera::RotateExp(const float angleYaw, const float anglePitch) {
    // Calculate side vector
    vec3 cameraSide = normalize(cross(mDirection, sCameraUp));

    // Yaw rotation around global up-axis (left-right)
    quat yawRotation = QAngleAxis(Deg2Rad(angleYaw), sCameraUp);

    // Pitch rotation around camera side-axis (up-down)
    quat pitchRotation = QAngleAxis(Deg2Rad(anglePitch), cameraSide);

    // Combine rotations
    quat rotation = normalize(yawRotation * pitchRotation);

    // Rotate camera direction
    mDirection = normalize(QRotate(rotation, mDirection));

    // Update camera transform
    this->MakeTransform();
}

void Camera::Rotate(const float angleX, const float angleY) {
    vec3 side = cross(mDirection, sCameraUp);
    quat pitchQ = QAngleAxis(Deg2Rad(angleY), side);
    quat headingQ = QAngleAxis(Deg2Rad(angleX), sCameraUp);
    //add the two quaternions
    quat temp = normalize(pitchQ * headingQ);
    // finally rotate our direction
    mDirection = normalize(QRotate(temp, mDirection));

    this->MakeTransform();
}

float Camera::GetNearPlane() const {
    return mNearZ;
}

float Camera::GetFarPlane() const {
    return mFarZ;
}

float Camera::GetFovY() const {
    return mFovY;
}

const mat4& Camera::GetProjection() const {
    return mProjection;
}

const mat4& Camera::GetTransform() const {
    return mTransform;
}

const vec3& Camera::GetPosition() const {
    return mPosition;
}

const vec3& Camera::GetDirection() const {
    return mDirection;
}

const vec3 Camera::GetUp() const {
    return vec3(mTransform[0][1], mTransform[1][1], mTransform[2][1]);
}

const vec3 Camera::GetSide() const {
    return vec3(mTransform[0][0], mTransform[1][0], mTransform[2][0]);
}

void Camera::MakeProjection() {
    const float aspect = static_cast<float>(mViewport.right - mViewport.left) / static_cast<float>(mViewport.bottom - mViewport.top);
    mProjection = MatProjection(Deg2Rad(mFovY), aspect, mNearZ, mFarZ);
}

void Camera::MakeTransform() {
    mTransform = MatLookAt(mPosition, mPosition + mDirection, sCameraUp);
}

void Camera::RotateYawPitchDeg(float yawDeltaDeg, float pitchDeltaDeg)
{
    SyncAnglesWithDirection();

    mYawDeg   += yawDeltaDeg;
    mPitchDeg  = std::clamp(mPitchDeg + pitchDeltaDeg, -89.0f, 89.0f);

    const float yaw   = Deg2Rad(mYawDeg);
    const float pitch = Deg2Rad(mPitchDeg);

    // Y up, forward starts at (0,0,1)
    vec3 f = normalize(vec3(
        cosf(pitch) * sinf(yaw), // x
        sinf(pitch),             // y (up/down)
        cosf(pitch) * cosf(yaw)  // z
        ));

    mDirection = f;
    MakeTransform();
}

void Camera::SyncAnglesWithDirection() {
    vec3 d = normalize(mDirection);
    mYawDeg   = Rad2Deg(std::atan2(d.x, d.z));                    // yaw: +Z forward
    mPitchDeg = Rad2Deg(std::asin(std::clamp(d.y, -1.0f, 1.0f))); // pitch
}


void Camera::Orbit(float deltaYaw, float deltaPitch, const vec3& center) {
    vec3 offset = mPosition - center;
    float radius = length(offset);
    if (radius < 0.001f) radius = 1.0f;

    // Convert to spherical coords
    float yaw = std::atan2(offset.x, offset.z);
    float pitch = std::asin(std::clamp(offset.y / radius, -1.0f, 1.0f));

    // Apply delta
    yaw -= Deg2Rad(deltaYaw); // Minus to match mouse dir
    pitch += Deg2Rad(deltaPitch);

    // Clamp pitch to avoid gimbal lock (vertical up/down)
    pitch = std::clamp(pitch, -1.5f, 1.5f); // ~ -85 to 85 degrees

    // Reconstruct position
    vec3 newOffset;
    newOffset.x = radius * std::cos(pitch) * std::sin(yaw);
    newOffset.y = radius * std::sin(pitch);
    newOffset.z = radius * std::cos(pitch) * std::cos(yaw);

    mPosition = center + newOffset;
    
    // Always look at center
    mDirection = normalize(center - mPosition);
    
    // Update internal state
    SyncAnglesWithDirection();
    MakeTransform();
}

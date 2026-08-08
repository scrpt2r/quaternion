--!native
--!optimize 2

--// Libs
local types = require(script.Parent.global.types)

--// Types
export type quaternion = types.quaternion
export type axis = types.axis
export type euler = types.euler

--// Constants
local QUATERNION_EPS = 1e-4
local PI = 3.14159265358979323846

--// Variables
local abs = math.abs
local sin = math.sin
local cos = math.cos
local acos = math.acos
local sqrt = math.sqrt
local atan2 = math.atan2
local asin = math.asin

--// Lib Decleration
local quaternion = {}

--// Lib Private Functions
function copysign(magnitude: number, sign: number): number
	local mag = abs(magnitude)

	if sign < 0 or (sign == 0 and 1 / sign < 0) then
		return -mag
	end

	return mag
end

function quaternion._set(w: number, v1: number, v2: number, v3: number, output: quaternion)
	assert(output ~= nil)

	output.w = w
	output.x = v1
	output.y = v2
	output.z = v3
end

function quaternion._copy(q: quaternion, output: quaternion)
	quaternion._set(q.w, q.x, q.y, q.z, output)
end

function quaternion._norm(q: quaternion): number
	assert(q ~= nil)
	return sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)
end

--// Lib Public Functions
function quaternion.identity(q: quaternion)
	assert(q ~= nil)
	quaternion._set(1, 0, 0, 0, q)
end

function quaternion.isEqual(q1: quaternion, q2: quaternion): boolean
	local equalW = abs(q1.w - q2.w) <= QUATERNION_EPS
	local equalV0 = abs(q1.x - q2.x) <= QUATERNION_EPS
	local equalV1 = abs(q1.y - q2.y) <= QUATERNION_EPS
	local equalV2 = abs(q1.z - q2.z) <= QUATERNION_EPS

	return equalW and equalV0 and equalV1 and equalV2
end

function quaternion.fromAxisAngle(axis: axis, angle: number, output: quaternion)
	assert(output ~= nil)

	output.w = cos(angle / 2.0)
	local c = sin(angle / 2.0)

	output.x = c * axis.x
	output.y = c * axis.y
	output.z = c * axis.z
end

function quaternion.toAxisAngle(q: quaternion, output: axis): number
	assert(output ~= nil)

	local angle = 2.0 * acos(q.w)
	local divider = sqrt(1.0 - q.w * q.w)

	if divider ~= 0 then
		output.x = q.x / divider
		output.y = q.y / divider
		output.z = q.z / divider
	else
		output.x = 1
		output.y = 0
		output.z = 0
	end

	return angle
end

function quaternion.fromXRotation(angle: number, output: quaternion)
	assert(output ~= nil)

	local axis = { x = 1.0, y = 0, z = 0 } :: axis
	quaternion.fromAxisAngle(axis, angle, output)
end

function quaternion.fromYRotation(angle: number, output: quaternion)
	assert(output ~= nil)

	local axis = { x = 0, y = 1.0, z = 0 } :: axis
	quaternion.fromAxisAngle(axis, angle, output)
end

function quaternion.fromZRotation(angle: number, output: quaternion)
	assert(output ~= nil)

	local axis = { x = 0, y = 0, z = 1.0 } :: axis
	quaternion.fromAxisAngle(axis, angle, output)
end

function quaternion.fromEulerZYX(eulerZYX: euler, output: quaternion)
	assert(output ~= nil)

	local cy = cos(eulerZYX.z * 0.5)
	local sy = sin(eulerZYX.z * 0.5)

	local cr = cos(eulerZYX.x * 0.5)
	local sr = sin(eulerZYX.x * 0.5)

	local cp = cos(eulerZYX.y * 0.5)
	local sp = sin(eulerZYX.y * 0.5)

	output.w = cy * cr * cp + sy * sr * sp
	output.x = cy * sr * cp - sy * cr * sp
	output.y = cy * cr * sp + sy * sr * cp
	output.z = sy * cr * cp - cy * sr * sp
end

function quaternion.toEulerZYX(q: quaternion, output: euler)
	assert(output ~= nil)

	local sinrCosp = 2.0 * (q.w * q.x + q.y * q.z)
	local cosrCosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
	output.x = atan2(sinrCosp, cosrCosp)

	local sinp = 2.0 * (q.w * q.y - q.z * q.x)
	if abs(sinp) >= 1 then
		output.y = copysign(PI / 2, sinp)
	else
		output.y = asin(sinp)
	end

	local sinyCosp = 2.0 * (q.w * q.z + q.x * q.y)
	local cosyCosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
	output.z = atan2(sinyCosp, cosyCosp)
end

function quaternion.conjugate(q: quaternion, output: quaternion)
	assert(output ~= nil)

	output.w = q.w
	output.x = -q.x
	output.y = -q.y
	output.z = -q.z
end

function quaternion.normalize(q: quaternion, output: quaternion)
	assert(output ~= nil)

	local len = quaternion._norm(q)
	quaternion._set(q.w / len, q.x / len, q.y / len, q.z / len, output)
end

function quaternion.multiply(q1: quaternion, q2: quaternion, output: quaternion)
	assert(output ~= nil)

	local w: number, x: number, y: number, z: number
	w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
	x = q1.x * q2.w + q1.w * q2.x + q1.y * q2.z - q1.z * q2.y
	y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
	z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w

	output.w = w
	output.x = x
	output.y = y
	output.z = z
end

function quaternion.rotate(q: quaternion, v: vector): vector
	assert(q ~= nil)
	local resX: number, resY: number, resZ: number

	local ww = q.w * q.w
	local xx = q.x * q.x
	local yy = q.y * q.y
	local zz = q.z * q.z

	local wx = q.w * q.x
	local wy = q.w * q.y
	local wz = q.w * q.z

	local xy = q.x * q.y
	local xz = q.x * q.z
	local yz = q.y * q.z

    --stylua: ignore
	resX = ww * v.x + 2 * wy * v.z - 2 * wz * v.y + xx * v.x + 
           2 * xy * v.y + 2 * xz * v.z - zz * v.x - yy * v.x
           
    --stylua: ignore
	resY = 2 * xy * v.x + yy * v.y + 2 * yz * v.z + 
           2 * wz * v.x - zz * v.y + ww * v.y - 
           2 * wx * v.z - xx * v.y

    --stylua: ignore
	resZ = 2 * xz * v.x + 2 * yz * v.y + zz * v.z -
           2 * wy * v.x - yy * v.z + 2 * wx * v.y -
           xx * v.z + ww * v.z

	return vector.create(resX, resY, resZ)
end

function quaternion.slerp(q1: quaternion, q2: quaternion, t: number, output: quaternion)
	local cosHalfTheta = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z

	if abs(cosHalfTheta) >= 1.0 then
		quaternion._copy(q1, output)
		return
	end

	local halfTheta = acos(cosHalfTheta)
	local sinHalfTheta = sqrt(1.0 - cosHalfTheta * cosHalfTheta)

	if abs(sinHalfTheta) < QUATERNION_EPS then
		local w: number, x: number, y: number, z: number
		w = (q1.w * 0.5 + q2.w * 0.5)
		x = (q1.x * 0.5 + q2.x * 0.5)
		y = (q1.y * 0.5 + q2.y * 0.5)
		z = (q1.z * 0.5 + q2.z * 0.5)

		output.w = w
		output.x = x
		output.y = y
		output.z = z
	else
		local ratioA = sin((1 - t) * halfTheta) / sinHalfTheta
		local ratioB = sin(t * halfTheta) / sinHalfTheta

		local w: number, x: number, y: number, z: number
		w = (q1.w * ratioA + q2.w * ratioB)
		x = (q1.x * ratioA + q2.x * ratioB)
		y = (q1.y * ratioA + q2.y * ratioB)
		z = (q1.z * ratioA + q2.z * ratioB)

		output.w = w
		output.x = x
		output.y = y
		output.z = z
	end
end

function quaternion.fromMatrix(
	R00: number, -- r11
	R01: number, -- r12
	R02: number, -- r13
	R10: number, -- r21
	R11: number, -- r22
	R12: number, -- r23
	R20: number, -- r31
	R21: number, -- r32
	R22: number, -- r33
	output: quaternion
)
	--[[
		Formula from: https://www.researchgate.net/publication/325950745_Accurate_Computation_of_Quaternions_from_Rotation_Matrices
		from 23 to 26 
	]]

	--[[
		q1: w
		q2: x
		q3: y
		q4: z
	]]

	--// q1
	if (R00 + R11 + R22) > 0 then
		output.w = sqrt(1 + R00 + R11 + R22) / 2.0
	else
		--stylua: ignore
		output.w = sqrt(
			(((R21 - R12) * (R21 - R12)) + 
			((R02 - R20) * (R02 - R20)) + 
			((R10 - R01) * (R10 - R01))) 
			/ (3 - R00 - R11 - R22)
		) / 2.0
	end

	--// q2
	if (R00 - R11 - R22) > 0 then
		output.x = sqrt(1 + R00 - R11 - R22) / 2.0
	else
		--stylua: ignore
		output.x = sqrt(
			(((R21 - R12) * (R21 - R12)) + 
			((R01 + R10) * (R01 + R10)) + 
			((R20 + R02) * (R20 + R02))) 
			/ (3 - R00 + R11 + R22)
		) / 2.0
	end
	output.x = copysign(output.x, R21 - R12)

	--// q3
	if (-R00 + R11 - R22) > 0 then
		output.y = sqrt(1 - R00 + R11 - R22) / 2.0
	else
		--stylua: ignore
		output.y = sqrt(
			(((R02 - R20) * (R02 - R20)) + 
			((R01 + R10) * (R01 + R10)) + 
			((R12 + R21) * (R12 + R21)))
			/ (3 + R00 - R11 + R22)
		) / 2.0
	end
	output.y = copysign(output.y, R02 - R20)

	--// q4
	if (-R00 - R11 + R22) > 0 then
		output.z = sqrt(1 - R00 - R11 + R22) / 2.0
	else
		--stylua: ignore
		output.z = sqrt(
			(((R10 - R01) * (R10 - R01)) + 
			((R20 + R02) * (R20 + R02)) + 
			((R21 + R12) * (R21 + R12)))
			/ (3 + R00 + R11 - R22)
		) / 2.0
	end
	output.z = copysign(output.z, R10 - R01)
end

function quaternion.fromCFrame(cframe: CFrame, output: quaternion)
	--stylua: ignore
	local _, _, _,
	R00: number, 
	R01: number, 
	R02: number, 
	R10: number, 
	R11: number, 
	R12: number, 
	R20: number, 
	R21: number, 
	R22: number 
	= cframe:GetComponents()

	quaternion.fromMatrix(R00, R01, R02, R10, R11, R12, R20, R21, R22, output)
end

return quaternion

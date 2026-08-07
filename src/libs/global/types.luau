--!native
--!optimize 2

--// Lib Decleration
local types = {}

--// Global Types
export type quaternion = {
	x: number,
	y: number,
	z: number,
	w: number,
}

export type axis = {
	x: number,
	y: number,
	z: number,
}

export type euler = {
	x: number,
	y: number,
	z: number,
}

export type XYZ = {
	x: number,
	y: number,
	z: number,
}

return types

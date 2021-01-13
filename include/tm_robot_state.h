#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#include <vector>
#include <string>
#include <mutex>
#include <functional>

template <typename T>
struct DataTableElement {
  bool isGetThisData = false;
  T data;
};

class TmDataTable;

class TmRobotState
{
	friend class TmDataTable;
private:
	TmDataTable *_data_table;

public:
	enum { DOF = 6 };

private:
	std::mutex mtx;

private:
	char _buf_[256];
	DataTableElement<unsigned char> _is_linked_;
	DataTableElement<unsigned char> _has_error_;
	DataTableElement<unsigned char> _is_proj_running_;
	DataTableElement<unsigned char> _is_proj_paused_;
	DataTableElement<unsigned char> _is_safeguard_A_triggered_;
	DataTableElement<unsigned char> _is_ESTOP_pressed_;
	DataTableElement<char> _camera_light_;

	DataTableElement<int> _error_code_;

	DataTableElement<float[DOF]> _joint_angle_;
	DataTableElement<float[6]> _flange_pose_;
	DataTableElement<float[6]> _tool_pose_;
	DataTableElement<float[6]> _tcp_frame_;
	DataTableElement<float> _tcp_mass_;
	DataTableElement<float[6]> _tcp_cog_;

	DataTableElement<float[3]> _tcp_force_vec_;
	DataTableElement<float> _tcp_force_;
	DataTableElement<float[6]> _tcp_speed_vec_;
	DataTableElement<float> _tcp_speed_;
	DataTableElement<float[DOF]> _joint_speed_;
	DataTableElement<float[DOF]> _joint_torque_;

	DataTableElement<int> _proj_speed_;
	DataTableElement<int> _ma_mode_;

	DataTableElement<char> _stick_play_pause_;

	DataTableElement<int> _robot_light_;

	DataTableElement<unsigned char[16]> _ctrller_DO_;
	DataTableElement<unsigned char[16]> _ctrller_DI_;
	DataTableElement<float[2]> _ctrller_AO_;
	DataTableElement<float[2]> _ctrller_AI_;
	DataTableElement<unsigned char[4]> _ee_DO_;
	DataTableElement<unsigned char[4]> _ee_DI_;
	DataTableElement<float[2]> _ee_AO_;
	DataTableElement<float[2]> _ee_AI_;
	DataTableElement<bool> isDataTableCorrect; 

private:
	DataTableElement<unsigned char> _is_linked;
	DataTableElement<unsigned char> _has_error;
	DataTableElement<unsigned char> _is_proj_running;
	DataTableElement<unsigned char> _is_proj_paused;
	DataTableElement<unsigned char> _is_safeguard_A_triggered;
	DataTableElement<unsigned char> _is_ESTOP_pressed;
	DataTableElement<char> _camera_light;

	DataTableElement<int> _error_code;
	DataTableElement<std::string> _error_content;

	DataTableElement<std::vector<double>> _joint_angle;	
	DataTableElement<std::vector<double>> _flange_pose;
	DataTableElement<std::vector<double>> _tool_pose;
	
	DataTableElement<std::vector<double>> _tcp_force_vec;
	DataTableElement<double >_tcp_force;
	DataTableElement<std::vector<double>> _tcp_speed_vec;
	DataTableElement<double> _tcp_speed;
	DataTableElement<std::vector<double>> _joint_speed;
	DataTableElement<std::vector<double>> _joint_torque;

	DataTableElement<std::vector<double>> _tcp_frame;
	DataTableElement<double> _tcp_mass;
	DataTableElement<std::vector<double>> _tcp_cog;

	DataTableElement<int> _proj_speed;
	DataTableElement<int> _ma_mode;

	DataTableElement<unsigned char> _stick_play_pause;

	DataTableElement<int> _robot_light;

	DataTableElement<std::vector<unsigned char>> _ctrller_DO;
	DataTableElement<std::vector<unsigned char>> _ctrller_DI;
	DataTableElement<std::vector<float>> _ctrller_AO;
	DataTableElement<std::vector<float>> _ctrller_AI;
	DataTableElement<std::vector<unsigned char>> _ee_DO;
	DataTableElement<std::vector<unsigned char>> _ee_DI;
	DataTableElement<std::vector<float>> _ee_AO;
	DataTableElement<std::vector<float>> _ee_AI;

private:
	std::function<size_t (void *, const char *, size_t)> _f_deserialize_item[2];
	std::function<size_t (const char *, size_t, bool)> _f_deserialize;
	struct ItemUpdate {
		void *dst;
		size_t func;
		enum { SKIP, UPDATE };
	};
	std::vector<ItemUpdate> _item_updates;
	void initial_data();

public:
	TmRobotState();
	~TmRobotState();

public:
	unsigned char is_linked() { return _is_linked.data; }
	unsigned char has_error() { return _has_error.data; }
    bool is_data_table_correct(){return isDataTableCorrect.data;}
	unsigned char is_project_running() { return _is_proj_running.data; }
	unsigned char is_project_paused() { return _is_proj_paused.data; }

	unsigned char is_safeguard_A() { return _is_safeguard_A_triggered.data; }
	unsigned char is_EStop() { return _is_ESTOP_pressed.data; }

	char camera_light() { return _camera_light.data; } // R/W

	int error_code() { return _error_code.data; }
	std::string error_content() { return _error_content.data; }

	std::vector<double> flange_pose() { return _flange_pose.data; }
	std::vector<double> joint_angle() { return _joint_angle.data; }
	std::vector<double> tool_pose() { return _tool_pose.data; }

	std::vector<double> tcp_force_vec() { return _tcp_force_vec.data; }
	double tcp_force() { return _tcp_force.data; }
	std::vector<double> tcp_speed_vec() { return _tcp_speed_vec.data; }
	double tcp_speed() { return _tcp_speed.data; }
	std::vector<double> joint_speed() { return _joint_speed.data; }
	std::vector<double> joint_torque() { return _joint_torque.data; }

	int project_speed() { return _proj_speed.data; }
	int ma_mode() { return _ma_mode.data; }

	unsigned char stick_play_pause() { return _stick_play_pause.data; } // R/W

	int robot_light() { return _robot_light.data; }

	std::vector<unsigned char> ctrller_DO() { return _ctrller_DO.data; }
	std::vector<unsigned char> ctrller_DI() { return _ctrller_DI.data; }
	std::vector<float> ctrller_AO() { return _ctrller_AO.data; }
	std::vector<float> ctrller_AI() { return _ctrller_AI.data; }

	std::vector<unsigned char> ee_DO() { return _ee_DO.data; }
	std::vector<unsigned char> ee_DI() { return _ee_DI.data; }
	//std::vector<float> ee_AO() { return _ee_AO; }
	std::vector<float> ee_AI() { return _ee_AI.data; }

public:
	void mtx_lock() { mtx.lock(); }
	void mtx_unlock() { mtx.unlock(); }

	std::vector<double> mtx_flange_pose();
	std::vector<double> mtx_joint_angle();
	std::vector<double> mtx_tool_pose();

	std::string mtx_error_content();

	std::vector<unsigned char> mtx_ctrller_DO();
	std::vector<unsigned char> mtx_ctrller_DI();
	std::vector<float> mtx_ctrller_AO();
	std::vector<float> mtx_ctrller_AI();

	std::vector<unsigned char> mtx_ee_DO();
	std::vector<unsigned char> mtx_ee_DI();
	//std::vector<float> mtx_ee_AO();
	std::vector<float> mtx_ee_AI();

private:
	static double meter(double mm)
	{
		return 0.001 * mm;
	}
	static double rad(double ang)
	{
		return (M_PI / 180.0) * ang;
	}
	template<typename T>
	static std::vector<double> meters(const T *mms, size_t size)
	{
		std::vector<double> rv(size);
		for (size_t i = 0; i < size; ++i) {
			rv[i] = meter(double(mms[i]));
		}
		return rv;
	}
	template<typename T>
	static std::vector<double> rads(const T *degs, size_t size)
	{
		std::vector<double> rv(size);
		for (size_t i = 0; i < size; ++i) {
			rv[i] = rad(double(degs[i]));
		}
		return rv;
	}
	template<typename T>
	static void si_pose(std::vector<double> &dst, T *src, size_t size = 6)
	{
		for (size_t i = 0; i < 3; ++i) { dst[i] = meter(double(src[i])); }
		for (size_t i = 3; i < size; ++i) { dst[i] = rad(double(src[i])); }
	}
	template<typename T>
    static DataTableElement<std::vector<double>> item_to_rad(DataTableElement<T> dataDeg, size_t size){
		std::vector<double> data(size);
        DataTableElement<std::vector<double>> rv;
		rv.data = data;
		for (size_t i = 0; i < size; ++i) {
			rv.data[i] = (rad(double(dataDeg.data[i])));
		}
		rv.isGetThisData = dataDeg.isGetThisData;
		return rv;
	}
	template<typename T>
	static void item_to_si_pose(DataTableElement<std::vector<double>> &dst, DataTableElement<T> *src, size_t size = 6){
		for (size_t i = 0; i < 3; ++i) { dst.data[i] = (rad(double(src->data[i]))); }
		for (size_t i = 3; i < size; ++i) { dst.data[i] = (rad(double(src->data[i]))); }
		dst.isGetThisData = src->isGetThisData;
	}
	template<typename T>
	static DataTableElement<std::vector<double>> item_to_meters(DataTableElement<T> *mms, size_t size)
	{   
		std::vector<double> data(size);
		DataTableElement<std::vector<double>> rv;
		rv.data = data;
		for (size_t i = 0; i < size; ++i) {
			rv.data[i] = (meter(double(mms->data[i])));
		}
		rv.isGetThisData = mms->isGetThisData;
		return rv;
	}
private:
	static size_t _deserialize_get_name(std::string &name, const char *data, size_t offset);
	static size_t _deserialize_skip(void *dst, const char *data, size_t offset);
	static size_t _deserialize_copy_wo_check(void *dst, const char *data, size_t offset);
	size_t _deserialize_first_time(const char *data, size_t size, bool lock);
	size_t _deserialize(const char *data, size_t size, bool use_mtx);
	void _deserialize_update(bool lock);

public:
	size_t deserialize(const char *data, size_t size)
	{
		return _f_deserialize(data, size, false);
	}
	size_t mtx_deserialize(const char *data, size_t size)
	{
		return _f_deserialize(data, size, true);
	}

	void print();
};

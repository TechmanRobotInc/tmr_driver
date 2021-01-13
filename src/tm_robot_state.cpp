#ifdef NO_INCLUDE_DIR
#include "tm_robot_state.h"
#include "tm_print.h"
#else
#include "../include/tm_robot_state.h"
#include "../include/tm_print.h"
#endif

#include <memory>
#include <cstring>
#include <sstream>
#include <iostream>

#include <map>

class TmDataTable
{
public:
	struct Item {
		void *dst;
		bool required;
		bool checked;
		enum { NOT_REQUIRE = 0 ,REQUIRED = 1 };
		Item() : dst(nullptr), required(true), checked(false) {};
		Item(void *d) : dst(d), required(true), checked(false) {};
		Item(void *d, bool r) : dst(d), required(r), checked(false) {};
	};
private:
	std::map<std::string, Item> _item_map;

public:
	TmDataTable(TmRobotState *rs)
	{
		print_debug("Create DataTable");

		_item_map.clear();
		//_item_map[""] = { Item:, &rs- };
		_item_map["Robot_Link"         ] = { &rs->_is_linked_.data };
		_item_map["Robot_Error"        ] = { &rs->_has_error_.data };
		_item_map["Project_Run"        ] = { &rs->_is_proj_running_.data };
		_item_map["Project_Pause"      ] = { &rs->_is_proj_paused_.data };
		_item_map["Safeguard_A"        ] = { &rs->_is_safeguard_A_triggered_.data};
		_item_map["ESTOP"              ] = { &rs->_is_ESTOP_pressed_.data };
		_item_map["Camera_Light"       ] = { &rs->_camera_light_.data };
		_item_map["Error_Code"         ] = { &rs->_error_code_.data };
		_item_map["Joint_Angle"        ] = { &rs->_joint_angle_.data };
		_item_map["Coord_Robot_Flange" ] = { &rs->_flange_pose_.data };
		_item_map["Coord_Robot_Tool"   ] = { &rs->_tool_pose_.data };
		_item_map["TCP_Force"          ] = { &rs->_tcp_force_vec_.data };
		_item_map["TCP_Force3D"        ] = { &rs->_tcp_force_.data };
		_item_map["TCP_Speed"          ] = { &rs->_tcp_speed_vec_.data };
		_item_map["TCP_Speed3D"        ] = { &rs->_tcp_speed_.data };
		_item_map["Joint_Speed"        ] = { &rs->_joint_speed_.data };
		_item_map["Joint_Torque"       ] = { &rs->_joint_torque_.data };
		_item_map["Project_Speed"      ] = { &rs->_proj_speed_.data };
		_item_map["MA_Mode"            ] = { &rs->_ma_mode_.data };
		_item_map["Robot_Light"        ] = { &rs->_robot_light_ };
		_item_map["Ctrl_DO0"           ] = { &rs->_ctrller_DO_.data[ 0] };
		_item_map["Ctrl_DO1"           ] = { &rs->_ctrller_DO_.data[ 1] };
		_item_map["Ctrl_DO2"           ] = { &rs->_ctrller_DO_.data[ 2] };
		_item_map["Ctrl_DO3"           ] = { &rs->_ctrller_DO_.data[ 3] };
		_item_map["Ctrl_DO4"           ] = { &rs->_ctrller_DO_.data[ 4] };
		_item_map["Ctrl_DO5"           ] = { &rs->_ctrller_DO_.data[ 5] };
		_item_map["Ctrl_DO6"           ] = { &rs->_ctrller_DO_.data[ 6] };
		_item_map["Ctrl_DO7"           ] = { &rs->_ctrller_DO_.data[ 7] };
		_item_map["Ctrl_DO8"           ] = { &rs->_ctrller_DO_.data[ 8],Item::NOT_REQUIRE };
		_item_map["Ctrl_DO9"           ] = { &rs->_ctrller_DO_.data[ 9],Item::NOT_REQUIRE };
		_item_map["Ctrl_DO10"          ] = { &rs->_ctrller_DO_.data[10],Item::NOT_REQUIRE };
		_item_map["Ctrl_DO11"          ] = { &rs->_ctrller_DO_.data[11],Item::NOT_REQUIRE };
		_item_map["Ctrl_DO12"          ] = { &rs->_ctrller_DO_.data[12],Item::NOT_REQUIRE };
		_item_map["Ctrl_DO13"          ] = { &rs->_ctrller_DO_.data[13],Item::NOT_REQUIRE };
		_item_map["Ctrl_DO14"          ] = { &rs->_ctrller_DO_.data[14],Item::NOT_REQUIRE };
		_item_map["Ctrl_DO15"          ] = { &rs->_ctrller_DO_.data[15],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI0"           ] = { &rs->_ctrller_DI_.data[ 0] };
		_item_map["Ctrl_DI1"           ] = { &rs->_ctrller_DI_.data[ 1] };
		_item_map["Ctrl_DI2"           ] = { &rs->_ctrller_DI_.data[ 2] };
		_item_map["Ctrl_DI3"           ] = { &rs->_ctrller_DI_.data[ 3] };
		_item_map["Ctrl_DI4"           ] = { &rs->_ctrller_DI_.data[ 4] };
		_item_map["Ctrl_DI5"           ] = { &rs->_ctrller_DI_.data[ 5] };
		_item_map["Ctrl_DI6"           ] = { &rs->_ctrller_DI_.data[ 6] };
		_item_map["Ctrl_DI7"           ] = { &rs->_ctrller_DI_.data[ 7] };
		_item_map["Ctrl_DI8"           ] = { &rs->_ctrller_DI_.data[ 8],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI9"           ] = { &rs->_ctrller_DI_.data[ 9],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI10"          ] = { &rs->_ctrller_DI_.data[10],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI11"          ] = { &rs->_ctrller_DI_.data[11],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI12"          ] = { &rs->_ctrller_DI_.data[12],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI13"          ] = { &rs->_ctrller_DI_.data[13],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI14"          ] = { &rs->_ctrller_DI_.data[14],Item::NOT_REQUIRE };
		_item_map["Ctrl_DI15"          ] = { &rs->_ctrller_DI_.data[15],Item::NOT_REQUIRE };
		_item_map["Ctrl_AO0"           ] = { &rs->_ctrller_AO_.data[ 0] };
		_item_map["Ctrl_AO1"           ] = { &rs->_ctrller_AO_.data[ 1] ,Item::NOT_REQUIRE};
		_item_map["Ctrl_AI0"           ] = { &rs->_ctrller_AI_.data[ 0] };
		_item_map["Ctrl_AI1"           ] = { &rs->_ctrller_AI_.data[ 1],Item::NOT_REQUIRE };
		_item_map["End_DO0"            ] = { &rs->_ee_DO_.data[0] };
		_item_map["End_DO1"            ] = { &rs->_ee_DO_.data[1] };
		_item_map["End_DO2"            ] = { &rs->_ee_DO_.data[2] };
		_item_map["End_DO3"            ] = { &rs->_ee_DO_.data[3] };
		_item_map["End_DI0"            ] = { &rs->_ee_DI_.data[0] };
		_item_map["End_DI1"            ] = { &rs->_ee_DI_.data[1] };
		_item_map["End_DI2"            ] = { &rs->_ee_DI_.data[2] };
		_item_map["End_DI3"            ] = { &rs->_ee_DI_.data[3],Item::NOT_REQUIRE };
		_item_map["End_AO0"            ] = { &rs->_ee_AO_.data[0],Item::NOT_REQUIRE };
		_item_map["End_AO1"            ] = { &rs->_ee_AO_.data[1],Item::NOT_REQUIRE };
		_item_map["End_AI0"            ] = { &rs->_ee_AI_.data[0] };
		_item_map["End_AI1"            ] = { &rs->_ee_AI_.data[1],Item::NOT_REQUIRE };
	}
	std::map<std::string, Item>  & get() { return _item_map; }
	std::map<std::string, Item>::iterator find(const std::string &name) { return _item_map.find(name); }
	std::map<std::string, Item>::iterator end() { return _item_map.end(); }
};
void TmRobotState::initial_data(){
    _joint_angle.data.assign(DOF, 0.0);
	_flange_pose.data.assign(6, 0.0);
	_tool_pose.data.assign(6, 0.0);
	_tcp_force_vec.data.assign(3, 0.0);
	_tcp_speed_vec.data.assign(6, 0.0);
	_joint_speed.data.assign(DOF, 0.0);
	_joint_torque.data.assign(DOF, 0.0);
	_tcp_frame.data.assign(6, 0.0);
	_tcp_cog.data.assign(6, 0.0);

	_ctrller_DO.data.assign(16, false);
	_ctrller_DI.data.assign(16, false);
	_ctrller_AO.data.assign(2, 0.0);
	_ctrller_AI.data.assign(2, 0.0);
	_ee_DO.data.assign(4, false);
	_ee_DI.data.assign(4, false);
	_ee_AO.data.assign(2, 0.0);
	_ee_AI.data.assign(2, 0.0);
}
TmRobotState::TmRobotState()
{
	print_debug("TmRobotState::TmRobotState");

	_data_table = new TmDataTable(this);

    initial_data();

	

	_f_deserialize_item[0] = std::bind(&TmRobotState::_deserialize_skip,
		std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	_f_deserialize_item[1] = std::bind(&TmRobotState::_deserialize_copy_wo_check,
		std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

	_f_deserialize = std::bind(&TmRobotState::_deserialize_first_time, this,
		std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
}
TmRobotState::~TmRobotState()
{
	print_debug("TmRobotState::~TmRobotState");
	delete _data_table;
}

std::vector<double> TmRobotState::mtx_flange_pose()
{
	std::vector<double> rv(_flange_pose.data.size());
	mtx_lock();
	rv = _flange_pose.data;
	mtx_unlock();
	return rv;
}
std::vector<double> TmRobotState::mtx_joint_angle()
{
	std::vector<double> rv(_joint_angle.data.size());
	mtx_lock();
	rv = _joint_angle.data;
	mtx_unlock();
	return rv;
}
std::vector<double> TmRobotState::mtx_tool_pose()
{
	std::vector<double> rv(_tool_pose.data.size());
	mtx_lock();
	rv = _tool_pose.data;
	mtx_unlock();
	return rv;
}
std::string TmRobotState::mtx_error_content()
{
	std::string rv;
	mtx_lock();
	rv = _error_content.data;
	mtx_unlock();
	return rv;
}
std::vector<unsigned char> TmRobotState::mtx_ctrller_DO()
{
	std::vector<unsigned char> rv;
	mtx_lock();
	rv = _ctrller_DO.data;
	mtx_unlock();
	return rv;
}
std::vector<unsigned char> TmRobotState::mtx_ctrller_DI()
{
	std::vector<unsigned char> rv;
	mtx_lock();
	rv = _ctrller_DI.data;
	mtx_unlock();
	return rv;
}
std::vector<float> TmRobotState::mtx_ctrller_AO()
{
	std::vector<float_t> rv;
	mtx_lock();
	rv = _ctrller_AO.data;
	mtx_unlock();
	return rv;
}
std::vector<float> TmRobotState::mtx_ctrller_AI()
{
	std::vector<float> rv;
	mtx_lock();
	rv = _ctrller_AO.data;
	mtx_unlock();
	return rv;
}
std::vector<unsigned char> TmRobotState::mtx_ee_DO()
{
	std::vector<unsigned char> rv;
	mtx_lock();
	rv = _ee_DO.data;
	mtx_unlock();
	return rv;
}
std::vector<unsigned char> TmRobotState::mtx_ee_DI()
{
	std::vector<unsigned char> rv;
	mtx_lock();
	rv = _ee_DI.data;
	mtx_unlock();
	return rv;
}
std::vector<float> TmRobotState::mtx_ee_AI()
{
	std::vector<float> rv;
	mtx_lock();
	rv = _ee_AI.data;
	mtx_unlock();
	return rv;
}

size_t TmRobotState::_deserialize_get_name(std::string &name, const char *data, size_t offset)
{
	size_t boffset = offset;
	unsigned short uslen; // 2 bytes

	// item name length
	memcpy(&uslen, data + boffset, 2);
	boffset += 2;
	// item name
	name = std::string{data + boffset, uslen};
	boffset += uslen;
	// skip item
	memcpy(&uslen, data + boffset, 2);
	boffset += 2 + uslen;
	return boffset;
}
size_t TmRobotState::_deserialize_skip(void *dst, const char *data, size_t offset)
{
	size_t boffset = offset;
	unsigned short uslen; // 2 bytes

	// skip item name
	memcpy(&uslen, data + boffset, 2);
	boffset += 2 + uslen;
	// skip item
	memcpy(&uslen, data + boffset, 2);
	boffset += 2 + uslen;

	if (dst) {}
	return boffset;
}
size_t TmRobotState::_deserialize_copy_wo_check(void *dst, const char *data, size_t offset)
{
	size_t boffset = offset;
	size_t bsize = 2;
	unsigned short uslen; // 2 bytes

	// skip item name
	memcpy(&uslen, data + boffset, bsize);
	boffset += bsize + uslen;
	// item data length
	memcpy(&uslen, data + boffset, bsize);
	boffset += bsize;
	// item data
	bsize = uslen;
	memcpy(dst, data + boffset, bsize);
	boffset += bsize;
	return boffset;
}

size_t TmRobotState::_deserialize_first_time(const char *data, size_t size, bool lock)
{
	size_t boffset = 0;
	size_t count = 0;
	size_t check_count = 0;
	size_t skip_count = 0;
	unsigned short uslen = 0; // 2 bytes
	std::string item_name;

	print_info("TM Flow DataTable Checked Item: ");
	_item_updates.clear();
	//_f_deserialize_item.clear();

	while (boffset < size && count < 100) {
		//boffset = _deserialize_get_name(item_name, data, boffset);
		// item name length
		memcpy(&uslen, data + boffset, 2);
		boffset += 2;
		// item name
		item_name = std::string{data + boffset, uslen};
		boffset += uslen;

		ItemUpdate update{ nullptr, ItemUpdate::SKIP };
		//std::function<size_t (void *, const char *, size_t)> func;
		auto iter = _data_table->find(item_name);
		if (iter != _data_table->end()) {
			update.dst = iter->second.dst;
			update.func = ItemUpdate::UPDATE;
			//func = std::bind(&RobotState::_deserialize_copy_wo_check, iter->second.dst,
			//	std::placeholders::_2, std::placeholders::_3);
			iter->second.checked = true;
			std::string msg = "- " + item_name + " - checked";
			print_info(msg.c_str());
			++check_count;
		}
		else {
			//func = std::bind(&RobotState::_deserialize_skip, nullptr,
			//	std::placeholders::_2, std::placeholders::_3);
			std::string msg = "- " + item_name + " - skipped";
			print_info(msg.c_str());
			++skip_count;
		}
		_item_updates.push_back({ update.dst, update.func });
		//_f_deserialize_item.push_back(func);

		// item data length
		memcpy(&uslen, data + boffset, 2);
		boffset += 2;
		if (update.func == ItemUpdate::SKIP) {
			// skip item
			boffset += uslen;
		}
		else {
			// item data
			memcpy(update.dst, data + boffset, uslen);
			boffset += uslen;
		}
		++count;
	}
	
	std::string msg = "Total " + std::to_string(_item_updates.size()) + " item," +
	std::to_string(check_count) + " checked, " + std::to_string(skip_count) + " skipped";
	print_info(msg.c_str());
	isDataTableCorrect.data = true;

	_deserialize_update(lock);

	for (auto iter : _data_table->get()) {
		if (iter.second.required && !iter.second.checked) {
			std::string msg = "Required item" + iter.first + " is NOT checked";
			isDataTableCorrect.data = false;
			print_error(msg.c_str());
		}
	}

	if(isDataTableCorrect.data ){
	    print_info("data table is correct!");
	} else{
        print_error("data table is not correct");
	}

	_f_deserialize = std::bind(&TmRobotState::_deserialize, this,
		std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

	return boffset;
}
size_t TmRobotState::_deserialize(const char *data, size_t size, bool use_mtx)
{
	size_t boffset = 0;

	for (auto &update : _item_updates) {
		boffset = _f_deserialize_item[update.func](update.dst, data, boffset);
	}


	_deserialize_update(use_mtx);

	if (boffset > size) {
	}
	return boffset;
}
void TmRobotState::_deserialize_update(bool lock) {
	// ---------------
	// update together
	// ---------------

	//const char c0 = 0;

	_is_linked = _is_linked_;
	_has_error = _has_error_;
	_is_proj_running = _is_proj_running_;
	_is_proj_paused = _is_proj_paused_;

	_is_safeguard_A_triggered = _is_safeguard_A_triggered_;
	_is_ESTOP_pressed = _is_ESTOP_pressed_;
	_camera_light = _camera_light_;
	_error_code = _error_code_;

	_proj_speed = _proj_speed_;
	_ma_mode = _ma_mode_;
	//_stick_play_pause = _stick_play_pause_;

	_robot_light = _robot_light_;

	if (lock) mtx_lock();
	{
		
		_joint_angle = item_to_rad(_joint_angle_, DOF);

		item_to_si_pose(_flange_pose, &_flange_pose_, 6);

		item_to_si_pose(_tool_pose, &_tool_pose_, 6);

		for (int i = 0; i < 3; ++i) { _tcp_force_vec.data[i] = double(_tcp_force_vec_.data[i]); }
		_tcp_force_vec.isGetThisData = _tcp_force_vec.isGetThisData;

		_tcp_force.data = double(_tcp_force_.data);
		_tcp_force.isGetThisData = _tcp_force_.isGetThisData;

		item_to_si_pose(_tcp_speed_vec, &_tcp_speed_vec_, 6);

		_joint_speed = item_to_rad(_joint_speed_, DOF);

		_joint_torque = item_to_meters(&_joint_torque_, DOF);

		// IO

		for (int i = 0; i < 8; ++i) { _ctrller_DO.data[i] = _ctrller_DO_.data[i]; }
		_ctrller_DO.isGetThisData = _ctrller_DO_.isGetThisData;
		for (int i = 0; i < 8; ++i) { _ctrller_DI.data[i] = _ctrller_DI_.data[i]; }
		_ctrller_DI.isGetThisData = _ctrller_DI.isGetThisData;
		for (int i = 0; i < 1; ++i) { _ctrller_AO.data[i] = _ctrller_AO_.data[i]; }
        _ctrller_AO.isGetThisData = _ctrller_AO.isGetThisData;
		for (int i = 0; i < 2; ++i) { _ctrller_AI.data[i] = _ctrller_AI_.data[i]; }
        _ctrller_AI.isGetThisData = _ctrller_AI.isGetThisData;
		for (int i = 0; i < 4; ++i) { _ee_DO.data[i] = _ee_DO_.data[i]; }
        _ee_DO.isGetThisData = _ee_DO.isGetThisData;
		for (int i = 0; i < 4; ++i) { _ee_DI.data[i] = _ee_DI_.data[i]; }
        _ee_DI.isGetThisData = _ee_DI.isGetThisData;
		//for (int i = 0; i < 1; ++i) { _ee_AO[i] = _ee_AO_[i]; }
		for (int i = 0; i < 1; ++i) { _ee_AI.data[i] = _ee_AI_.data[i]; }
        _ee_AI.isGetThisData = _ee_AI.isGetThisData;
	}
	if (lock) mtx_unlock();
}

void TmRobotState::print()
{
	mtx_lock();
	std::cout << "Robot_Link=" << _is_linked.data << "\n";
	std::cout << "Robot_Error=" << _has_error.data << "\n";
	std::cout << "Project_Run=" << _is_proj_running.data << "\n";
	std::cout << "Project_Pause=" << _is_proj_running.data << "\n";
	std::cout << "Safetyguard_A=" << _is_safeguard_A_triggered.data << "\n";
	std::cout << "ESTOP=" << _is_ESTOP_pressed.data << "\n";

	std::cout << "Joint_Angle={";
	for (auto &val : _joint_angle.data) { std::cout << val << ", "; }
	std::cout << "}\n";

	std::cout << "Coord_Robot_Tool0={";
	for (auto &val : _flange_pose.data) { std::cout << val << ", "; }
	std::cout << "}\n";

	std::cout << "Coord_Robot_Tool={";
	for (auto &val : _tool_pose.data) { std::cout << val << ", "; }
	std::cout << "}\n";

	std::cout << "Error_Code=" << _error_code.data << "\n";
	std::cout << "Error_Content=" << _error_content.data << "\n";
	mtx_unlock();
}

#ifdef NO_INCLUDE_DIR
#include "tm_svr_communication.h"
#include "tm_print.h"
#else
#include "../include/tm_svr_communication.h"
#include "../include/tm_print.h"
#endif

#include <functional>

//
// TmSvrCommunication
//

TmSvrCommunication::TmSvrCommunication(const std::string &ip,
	int recv_buf_len, std::condition_variable *cv)
	: TmCommunication(ip.c_str(), 5891, recv_buf_len)
{
	if (cv) {
		_cv = cv;
		_has_thread = true;
	}
}
TmSvrCommunication::~TmSvrCommunication()
{
	halt();
}

bool TmSvrCommunication::start(int timeout_ms)
{
	if (socket_description() == 6188)
	{
		print_info("TM_SVR: start (fake)");
		if (_has_thread) {
			// start thread
			_recv_thread = std::thread(std::bind(&TmSvrCommunication::thread_function, this));
		}
		return true;
	}

	halt();
	print_info("TM_SVR: start");

	bool rb = Connect(timeout_ms);
	//if (!rb) return rb; // ? start thread anyway

	if (_has_thread) {
		// start thread
		_recv_thread = std::thread(std::bind(&TmSvrCommunication::thread_function, this));
	}
	return rb;
}
void TmSvrCommunication::halt()
{
	if (socket_description() == 6188)
	{
		print_info("TM_SVR: halt (fake)");
		if (_has_thread) {
			_keep_thread_alive = false;
			if (_recv_thread.joinable()) {
				_recv_thread.join();
			}
		}
		return;
	}
	if (_has_thread) {
		_keep_thread_alive = false;
		if (_recv_thread.joinable()) {
			_recv_thread.join();
		}
	}
	if (is_connected()) {
		print_info("TM_SVR: halt");
		//if (_has_thread) {
		//	_keep_thread_alive = false;
		//	if (_recv_thread.joinable()) {
		//		_recv_thread.join();
		//	}
		//}
		Close();
	}
}

TmCommRC TmSvrCommunication::send_content(const std::string &id, TmSvrData::Mode mode, const std::string &content)
{
	std::string cntt = content;
	TmSvrData cmd{ id, mode, cntt.data(), cntt.size(), TmSvrData::SrcType::Shallow };
	TmPacket pack{ cmd };
	return send_packet_all(pack);
}
TmCommRC TmSvrCommunication::send_content_str(const std::string &id, const std::string &content)
{
	std::string cntt = content;
	TmSvrData cmd{ id, TmSvrData::Mode::STRING, cntt.data(), cntt.size(), TmSvrData::SrcType::Shallow };
	TmPacket pack{ cmd };
	return send_packet_all(pack);
}
TmCommRC TmSvrCommunication::send_stick_play()
{
	return send_content_str("Play", "Stick_PlayPause=1");
}

void TmSvrCommunication::thread_function()
{
	print_info("TM_SVR: thread begin");
	_keep_thread_alive = true;
	while (_keep_thread_alive) {
		bool reconnect = false;
		if (!recv_init()) {
			print_info("TM_SVR: is not connected");
		}
		while (_keep_thread_alive && is_connected() && !reconnect) {
			TmCommRC rc = tmsvr_function();
			_updated = true;
			_cv->notify_all();

			switch (rc) {
			case TmCommRC::ERR:
			case TmCommRC::NOTREADY:
			case TmCommRC::NOTCONNECT:
				print_info("TM_SVR: rc=%d", int(rc));
				reconnect = true;
				break;
			default: break;
			}
		}
		Close();
		reconnect_function();
	}
	Close();
	print_info("TM_SVR: thread end");
}
void TmSvrCommunication::reconnect_function()
{
	if (!_keep_thread_alive) return;
	if (_reconnect_timeval_ms <= 0) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	print_info("TM_SVR: reconnect in ");
	int cnt = 0;
	while (_keep_thread_alive && cnt < _reconnect_timeval_ms) {
		if (cnt % 500 == 0) {
			print_info("%.1f sec...", 0.001 * (_reconnect_timeval_ms - cnt));
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		++cnt;
	}
	if (_keep_thread_alive && _reconnect_timeval_ms >= 0) {
		print_info("0 sec\nTM_SVR: connect(%dms)...", _reconnect_timeout_ms);
		Connect(_reconnect_timeout_ms);
	}
}
TmCommRC TmSvrCommunication::tmsvr_function()
{
	TmCommRC rc;
	int n;
	rc = recv_spin_once(1000, &n);
	if (rc != TmCommRC::OK) {
		return rc;
	}
	std::vector<TmPacket> &pack_vec = packet_list();

	for (auto &pack : pack_vec) {
		if (pack.type == TmPacket::Header::CPERR) {
			print_info("TM_SVR: CPERR");
			err_data.set_CPError(pack.data.data(), pack.data.size());
			print_error(err_data.error_code_str().c_str());
		}
		else if (pack.type == TmPacket::Header::TMSVR) {
			
			err_data.error_code(TmCPError::Code::Ok);

			TmSvrData::build_TmSvrData(data, pack.data.data(), pack.data.size(), TmSvrData::SrcType::Shallow);
			
			if (data.is_valid()) {
				switch (data.mode()) {
				case TmSvrData::Mode::RESPONSE:
					print_info("TM_SVR: RESPONSE (%s): [%d]: %s", data.transaction_id().c_str(),
						(int)(data.error_code()), std::string(data.content(), data.content_len()).c_str());
					break;
				case TmSvrData::Mode::BINARY:
					state.mtx_deserialize(data.content(), data.content_len());
					break;
				case TmSvrData::Mode::READ_STRING:
					print_info("TM_SVR: READ_STRING (%s): %s", data.transaction_id().c_str(),
						std::string(data.content(), data.content_len()).c_str());
					break;
				default:
					print_info("TM_SVR: (%s): invalid mode (%d)", data.transaction_id().c_str(), (int)(data.mode()));
					break;
				}
			}
			else {
				print_info("TM_SVR: invalid data");
			}
		}
		else {
			print_info("TM_SVR: invalid header");
		}
	}
	return rc;
}

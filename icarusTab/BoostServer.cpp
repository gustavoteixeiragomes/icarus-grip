//
// This software is derivated from third_party_lib.cpp at http://www.boost.org/doc/libs/1_53_0/doc/html/boost_asio/example/nonblocking/third_party_lib.cpp
// ~~~~~~~~~~~~~~~~~~~
// 
// This file: Copyright (c) 2013 Gustavo Gomes (github at guvux dot com dot br)
// Original file: Copyright (c) 2003-2008 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include "BoostServer.h"

// Convert a string into an aray of size MAX_BUFFER
boost::array<char, MAX_BUFFER> BoostServer::session::string2array(std::string str) {
	boost::array<char, MAX_BUFFER> dataTemp;
	int size = str.size();
		
	if (size > MAX_BUFFER) {
		size = MAX_BUFFER; 
	}
	for (int i=0; i<size; i++) {
		dataTemp.c_array()[i] = str[i];
	}
	dataTemp.c_array()[size] = '\0';
	return dataTemp;
}
			
// Notify that library that it should perform its read line operation.
void BoostServer::session::do_read_line(boost::system::error_code& ec) {
	if (std::size_t len = socket_.read_some(boost::asio::buffer(data_), ec)) {
		// Append to request
		std::string strTemp = data_.data();
		std::string request = handle_.getRequest();
		request += strTemp.substr(0, len);
		handle_.setRequest(request);
			
		// Verify end of msg
		if (data_.data()[len-1] == '\n') {
			// Adjust the request string
			request = request.substr(0, request.size()-1);
			handle_.setRequest(request);
			
			// Call function to handle with the request
			handle_.execute();

			// Adjust the answer string
			std::string answer = handle_.getAnswer();
			answer.append("\n");

			data_ = string2array(answer);
			write_buffer_ = boost::asio::buffer(data_, answer.size());
			handle_.clear();
			state_ = writing;
		}
	}
}

// Notify that library that it should perform its read operation.
void BoostServer::session::do_read(boost::system::error_code& ec) {
	if (std::size_t len = socket_.read_some(boost::asio::buffer(data_), ec)) {
		write_buffer_ = boost::asio::buffer(data_, len);
		state_ = writing;
	}
}

// Notify that library that it should perform its write operation.
void BoostServer::session::do_write(boost::system::error_code& ec) {
	if (std::size_t len = socket_.write_some(boost::asio::buffer(write_buffer_), ec)) {
		write_buffer_ = write_buffer_ + len;
		state_ = boost::asio::buffer_size(write_buffer_) > 0 ? writing : reading;
	}
}

tcp::socket& BoostServer::connection::socket() {
	return socket_;
}

void BoostServer::connection::start() {
	// Put the socket into non-blocking mode.
	tcp::socket::non_blocking_io non_blocking_io(true);
	socket_.io_control(non_blocking_io);
	
	start_operations();
}

void BoostServer::connection::start_operations() {
	// Start a read operation if the library wants one.
	if (session_impl_.want_read() && !read_in_progress_) {
		read_in_progress_ = true;
		socket_.async_read_some(
			boost::asio::null_buffers(),
			boost::bind(&connection::handle_read,
			shared_from_this(),
			boost::asio::placeholders::error));
	}

	// Start a write operation if the library wants one.
	if (session_impl_.want_write() && !write_in_progress_) {
		write_in_progress_ = true;
		socket_.async_write_some(
			boost::asio::null_buffers(),
			boost::bind(&connection::handle_write,
			shared_from_this(),
			boost::asio::placeholders::error));
	}
}

void BoostServer::connection::handle_read(boost::system::error_code ec) {
	read_in_progress_ = false;

	// Notify library that it can perform a read.
	if (!ec) {
		session_impl_.do_read_line(ec);
	}

	// The library successfully performed a read on the socket.
	// Start new read or write operations based on what it now wants.
	if (!ec || ec == boost::asio::error::would_block) {
		start_operations();
	}

	// Otherwise, an error occurred. Closing the socket cancels any outstanding
	// asynchronous read or write operations. The connection object will be
	// destroyed automatically once those outstanding operations complete.
	else {
		socket_.close();
	}
}

void BoostServer::connection::handle_write(boost::system::error_code ec) {
	write_in_progress_ = false;

	// Notify library that it can perform a write.
	if (!ec) {
		session_impl_.do_write(ec);
	}

	// The library successfully performed a write on the socket.
	// Start new read or write operations based on what it now wants.
	if (!ec || ec == boost::asio::error::would_block) {
		start_operations();
	}

	// Otherwise, an error occurred. Closing the socket cancels any outstanding
	// asynchronous read or write operations. The connection object will be
	// destroyed automatically once those outstanding operations complete.
	else {
		socket_.close();
	}
}

void BoostServer::server::start_accept() {
	connection::pointer new_connection = connection::create(acceptor_.get_io_service());

	acceptor_.async_accept(new_connection->socket(), boost::bind(&server::handle_accept, this, new_connection, boost::asio::placeholders::error));
}

void BoostServer::server::handle_accept(connection::pointer new_connection,
    const boost::system::error_code& error) {
	if (!error) {
		new_connection->start();
		start_accept();
	}
}
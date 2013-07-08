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
#pragma once
#ifndef __BOOST_SERVER__
	#define __BOOST_SERVER__

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
	#ifndef _WIN32_WINNT
		#define _WIN32_WINNT 0x0501
	#endif
#endif
// Define max size of read buffer for socket
#define MAX_BUFFER 128
#define SERVER_PORT 6003

#include "HandleInterface.h"
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
//#include <simulation/World.h>

using boost::asio::ip::tcp;

namespace BoostServer {

	// Implementation of a library that wants to perform read and write
	// operations directly on a socket. It needs to be polled to determine whether
	// it requires a read or write operation, and notified when the socket is ready
	// for reading or writing.
	class session {
		public:
			session(tcp::socket& socket)
			: socket_(socket),
			state_(reading) {}
		// Returns true if the library wants to be notified when the socket is ready for reading.
		bool want_read() const {
			return state_ == reading;
		}

		// Convert a string into an aray of size MAX_BUFFER
		boost::array<char, MAX_BUFFER> string2array(std::string str);
		
		// Notify that library that it should perform its read line operation.
		void do_read_line(boost::system::error_code& ec);

		// Notify that library that it should perform its read operation.
		void do_read(boost::system::error_code& ec);

		// Returns true if library wants to be notified when the
		// socket is ready for writing.
		bool want_write() const {
			return state_ == writing;
		}

		// Notify that library that it should perform its write operation.
		void do_write(boost::system::error_code& ec);

		handleInterface::Handle handle_;

	private:
		tcp::socket& socket_;
		enum { reading, writing } state_;
		boost::array<char, MAX_BUFFER> data_;
		boost::asio::const_buffer write_buffer_;
	};

	// The glue between asio's sockets and the boost-server library.
	class connection : public boost::enable_shared_from_this<connection> {
		public:
			typedef boost::shared_ptr<connection> pointer;

			static pointer create(boost::asio::io_service& io_service) {
				return pointer(new connection(io_service));
			};
			tcp::socket& socket();
			//void setWorld(simulation::World* mWorld);
			void start();

		private:
			connection(boost::asio::io_service& io_service)
				: socket_(io_service),
				session_impl_(socket_),
				read_in_progress_(false),
				write_in_progress_(false) {}

			void start_operations();
			void handle_read(boost::system::error_code ec);
			void handle_write(boost::system::error_code ec);

		private:
			tcp::socket socket_;
			session session_impl_;
			bool read_in_progress_;
			bool write_in_progress_;
	};

	class server {
		public:
			server(boost::asio::io_service& io_service, unsigned short port)
				: acceptor_(io_service, tcp::endpoint(tcp::v4(), port)) {
					start_accept();
			}
			//void setWorld(simulation::World* mWorld);

		private:
			void start_accept();
			void handle_accept(connection::pointer new_connection, const boost::system::error_code& error);
			tcp::acceptor acceptor_;
			//simulation::World* world_;
	};

} // namespace BoostServer

#endif
/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2020 Universidade Federal de Minas Gerais
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Igor Castejon F. e Castro <igorcastejon@dcc.ufmg.br>
 */

#ifndef UWGRE_DATASTRUCTURE_H
#define UWGRE_DATASTRUCTURE_H
// mess types
#define JOIN_REQ 1
#define JOIN_REPLY 2
#define NB_SET_REQ 3
#define NB_SET_REPLY 4
#define NB_DISCOVER_REQ 5
#define NB_DISCOVER_REPLY 6
#define UPDATE_NEIGH_POSITION 10
#define VPOD_INITIALIZE 11
#define MDT_NEIGH_JOIN 12
#define MDT_INITIALIZE 13
#define NB_SET_REQ_MAINTENANCE 14
#define NB_SET_REPLY_MAINTENANCE 15
#define NB_SET_REQ_NOTIFICATION 16
#define MDT_DFS_INITIALIZE 17
#define MDT_DFS_COMEBACK 18
#define UWGRE 20
#define MDT 21
#define SYMMETRIC_ETX 22
#define NULL 999

#endif
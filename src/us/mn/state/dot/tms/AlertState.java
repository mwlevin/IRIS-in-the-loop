/*
 * IRIS -- Intelligent Roadway Information System
 * Copyright (C) 2021  Minnesota Department of Transportation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
package us.mn.state.dot.tms;

/**
 * Alert States
 *
 * The ordinal values correspond to the records in the iris.alert_state look-up
 * table.
 *
 * @author Douglas Lau
 */
public enum AlertState {
	PENDING,     // 0  Pending approval by user
	ACTIVE,      // 1  Scheduled and active
	CLEARED,     // 2  Expired or cleared by user
	ACTIVE_REQ,  // 3  User activate request
	CLEARED_REQ; // 4  User cleared request

	/** Values array */
	static private final AlertState[] VALUES = values();

	/** Get a AlertState from an ordinal value */
	static public AlertState fromOrdinal(int o) {
		return (o >= 0 && o < VALUES.length) ? VALUES[o] : null;
	}
}

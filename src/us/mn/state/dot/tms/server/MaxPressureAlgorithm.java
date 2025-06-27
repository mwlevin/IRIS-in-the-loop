/*
 * IRIS -- Intelligent Roadway Information System
 * Copyright (C) 2001-2025  Minnesota Department of Transportation
 * Copyright (C) 2011-2012  University of Minnesota Duluth (NATSRL)
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
package us.mn.state.dot.tms.server;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import us.mn.state.dot.sched.DebugLog;
import us.mn.state.dot.sched.TimeSteward;
import us.mn.state.dot.tms.EventType;
import us.mn.state.dot.tms.GeoLoc;
import us.mn.state.dot.tms.LaneCode;
import us.mn.state.dot.tms.MeterQueueState;
import us.mn.state.dot.tms.R_NodeHelper;
import us.mn.state.dot.tms.R_NodeType;
import static us.mn.state.dot.tms.R_NodeType.ENTRANCE;
import static us.mn.state.dot.tms.R_NodeType.STATION;
import us.mn.state.dot.tms.RampMeterHelper;
import static us.mn.state.dot.tms.RampMeterHelper.filterRate;
import static us.mn.state.dot.tms.RampMeterHelper.getMaxRelease;
import us.mn.state.dot.tms.SystemAttrEnum;
import static us.mn.state.dot.tms.server.Constants.FEET_PER_MILE;
import static us.mn.state.dot.tms.server.Constants.MISSING_DATA;
import us.mn.state.dot.tms.units.Interval;
import static us.mn.state.dot.tms.units.Interval.HOUR;
import us.mn.state.dot.tms.server.event.MeterEvent;
import us.mn.state.dot.tms.server.maxpressure.CCSamplerSet;
import static us.mn.state.dot.tms.units.Interval.HOUR;

/**
 * Density-based Adaptive Metering Algorithm.
 *
 * I'm going to copy KAdaptive because it has a lot of what I need.
 * There doesn't seem to be an easy way to inherit from KAdaptive because of its use of child classes and private methods.
 * I would need to replace some of KAdaptive.MeterState methods.
 */
public class MaxPressureAlgorithm implements MeterAlgorithmState {

	/** Enum for minimum limit control */
	enum MinimumRateLimit {
		passage_fail,
		storage_limit,
		wait_limit,
		target_min,
		backup_limit,
	};
        
        

	/** Algorithm debug log */
	static private final DebugLog ALG_LOG = new DebugLog("kadaptive");

	/** Number of seconds for one time step */
	static private final int STEP_SECONDS = 30;

	/** Number of milliseconds for one time step */
	static private final int PERIOD_MS =
		(int) new Interval(STEP_SECONDS).ms();
        
	/** Calculate steps per hour */
	static private final double STEP_HOUR =
		new Interval(STEP_SECONDS).per(HOUR);

	/** Critical density (vehicles / mile) */
	static private final int K_CRIT = 37;

	/** Desired density (vehicles / mile) */
	static private final double K_DES = K_CRIT * 0.9;

	/** Low density (vehicles / mile) */
	static private final double K_LOW = K_CRIT * 0.75;

	/** Jam density (vehicles / mile) */
	static private final int K_JAM = 180;

	/** Ramp queue jam density (vehicles / mile) */
	static private final int K_JAM_RAMP = 140;

	/** Ramp queue jam density (vehicles per foot) */
	static private final float JAM_VPF = (float) K_JAM_RAMP / FEET_PER_MILE;

	/** Seconds to average segment density for start metering check */
	static private final int START_SECS = 120;

	/** Seconds to average segment density for stop metering check */
	static private final int STOP_SECS = 600;

	/** Seconds to average segment density for restart metering check */
	static private final int RESTART_SECS = 300;

	/** Maximum number of time steps needed for sample history */
	static private final int MAX_STEPS = steps(Math.max(Math.max(START_SECS,
		STOP_SECS), RESTART_SECS));

	/** Minutes before end of period to disallow early metering */
	static private final int EARLY_METER_END_MINUTES = 30;

	/** Minutes to flush meter before stop metering */
	static private final int FLUSH_MINUTES = 2;

	/** Distance threshold for upstream station to meter association */
	static private final float UPSTREAM_STATION_MILES = 1.0f;

	/** Distance threshold for downstream station to meter association */
	static private final int DOWNSTREAM_STATION_FEET = 500;

	/** Maximum segment length */
	static private final float SEGMENT_LENGTH_MILES = 3.0f;

	/** Number of seconds to store demand accumulator history */
	static private final int DEMAND_ACCUM_SECS = 600;

	/** Queue occupancy override threshold */
	static private final int QUEUE_OCC_THRESHOLD = 25;

	/** Number of seconds queue must be empty before resetting green */
	static private final int QUEUE_EMPTY_RESET_SECS = 60;

	/** Threshold to determine when queue is empty */
	static private final int QUEUE_EMPTY_THRESHOLD = -5;

	/** Ratio for max rate to target rate */
	static private final float TARGET_MAX_RATIO = 1.25f;

	/** Ratio for max rate to target rate while flushing */
	static private final float TARGET_MAX_RATIO_FLUSHING = 1.5f;

	/** Ratio for min rate to target rate */
	static private final float TARGET_MIN_RATIO = 0.75f;

	/** Base percentage for backup minimum limit */
	static private final float BACKUP_LIMIT_BASE = 0.5f;

	/** Ratio for target waiting time to max wait time */
	static private final float WAIT_TARGET_RATIO = 0.75f;

	/** Ratio for target storage to max storage */
	static private final float STORAGE_TARGET_RATIO = 0.75f;
        
        /* this needs to be updated */
        static private final double AVG_VEH_LEN = 27.6;
        
        
        static private final int NUM_PRESSURE_INTERVAL = 10;
        
        // jam density
        static private final double K = 5280.0/AVG_VEH_LEN;

	/** Calculate the number of steps for an interval */
	static private int steps(int seconds) {
		float secs = seconds;
		return Math.round(secs / STEP_SECONDS);
	}

	/** Convert step vehicle count to flow rate.
	 * @param v Vehicle count to convert.
	 * @param n_steps Number of time steps of vehicle count.
	 * @return Flow rate (vehicles / hour) */
	static private int flowRate(float v, int n_steps) {
		if (v >= 0) {
			Interval period = new Interval(n_steps * STEP_SECONDS);
			float hour_frac = period.per(HOUR);
			return Math.round(v * hour_frac);
		} else
			return MISSING_DATA;
	}

	/** Convert single step vehicle count to flow rate.
	 * @param v Vehicle count to convert.
	 * @return Flow rate (vehicles / hour), or null for missing data. */
	static private Double flowRate(float v) {
		return (v >= 0) ? (v * STEP_HOUR) : null;
	}

	/** Convert flow rate to vehicle count for a given period.
	 * @param flow Flow rate to convert (vehicles / hour).
	 * @param per_sec Period for vehicle count (seconds).
	 * @return Vehicle count over given period. */
	static private float vehCountPeriod(int flow, int per_sec) {
		if (flow >= 0 && per_sec > 0) {
			float hour_frac = HOUR.per(new Interval(per_sec));
			return flow * hour_frac;
		} else
			return MISSING_DATA;
	}

	/** Check if density is below "low" threshold */
	static private boolean isDensityLow(Double k) {
		return (k != null) && (k < K_LOW);
	}

	/** States for all K adaptive algorithms */
	static private HashMap<String, MaxPressureAlgorithm> ALL_ALGS =
		new HashMap<String, MaxPressureAlgorithm>();

	/** Create algorithm state for a meter */
	static public MaxPressureAlgorithm createState(RampMeterImpl meter) {
		Corridor c = meter.getCorridor();
                
                //System.out.println("create state "+meter.getName()+" "+c);
		if (c != null) {
			MaxPressureAlgorithm alg = lookupAlgorithm(c);
			if (alg.createMeterState(meter))
				return alg;
		}
		return null;
	}

	/** Lookup an algorithm for a corridor */
	static private MaxPressureAlgorithm lookupAlgorithm(Corridor c) {
		MaxPressureAlgorithm alg = ALL_ALGS.get(c.getName());
		if (null == alg) {
			alg = new MaxPressureAlgorithm(c);
			alg.log("adding");
			ALL_ALGS.put(c.getName(), alg);
		}
		return alg;
	}

	/** Process one interval for all K adaptive algorithm states */
	static public void processAllStates() {
		long stamp = DetectorImpl.calculateEndTime(PERIOD_MS);
		Iterator<MaxPressureAlgorithm> it =
			ALL_ALGS.values().iterator();
		while (it.hasNext()) {
			MaxPressureAlgorithm alg = it.next();
			alg.updateStations(stamp);
			if (alg.isDone()) {
				alg.log("isDone: removing");
				it.remove();
			}
		}
	}

	/** Metering corridor */
	private final Corridor corridor;

	/** Hash map of ramp meter states */
	private final HashMap<String, MeterState> meter_states =
		new HashMap<String, MeterState>();

	/** All entrance / station nodes on corridor */
	private final ArrayList<Node> nodes;

	/** Create a new MaxPressureAlgorithm */
	private MaxPressureAlgorithm(Corridor c) {
		corridor = c;
                
                System.out.println("construct max pressure");
		nodes = createNodes();
		debug();
	}

	/** Create nodes from corridor structure */
	private ArrayList<Node> createNodes() {
		NFinder finder = new NFinder();
		corridor.findActiveNode(finder);
		return finder.nodes;
	}

	/** Node finder */
	private class NFinder implements Corridor.NodeFinder {
		private ArrayList<Node> nodes = new ArrayList<Node>();
		public boolean check(float m, R_NodeImpl rnode) {
			Node n = createNode(rnode, m);
			if (n != null)
				nodes.add(n);
			return false;
		}
	}

	/** Create one node */
	private Node createNode(R_NodeImpl rnode, float mile) {
                //System.out.println(rnode.getName()+" "+mile+" "+ R_NodeType.fromOrdinal(rnode.getNodeType())+" "+(R_NodeType.fromOrdinal(rnode.getNodeType())==STATION));
                
                System.out.println(rnode.getName()+" "+mile+" "+rnode.getDetectors().length);
                
		switch (R_NodeType.fromOrdinal(rnode.getNodeType())) {
		case ENTRANCE:
			return new EntranceNode(rnode, mile);
		case STATION:
			StationImpl stat = rnode.getStation();
			if (stat != null && stat.getActive())
				return new StationNode(rnode, mile, stat);
		default:
			return null;
		}
	}

	/** Debug corridor structure */
	private void debug() {
		log("-------- Corridor Structure --------");
		for (Node n : nodes)
			log(n.toString());
	}

	/** Log one message */
	private void log(String msg) {
		if (ALG_LOG.isOpen())
			ALG_LOG.log(corridor.getName() + ": " + msg);
	}

	/** Validate algorithm state for a meter */
	@Override
	public void validate(RampMeterImpl meter) {
		MeterState ms = getMeterState(meter);
		if (ms != null) {
			ms.validate();
			if (ALG_LOG.isOpen())
				log(ms.toString());
			ms.logMeterEvent();
		} else if (ALG_LOG.isOpen())
			log("No state for " + meter.getName());
	}

	/** Get meter queue state enum value */
	@Override
	public MeterQueueState getQueueState(RampMeterImpl meter) {
		MeterState ms = getMeterState(meter);
		return (ms != null)
		      ? ms.getQueueState()
		      : MeterQueueState.UNKNOWN;
	}

	/** Get the meter state for a given ramp meter */
	private MeterState getMeterState(RampMeterImpl meter) {
		if (meter.getCorridor() == corridor)
			return meter_states.get(meter.getName());
		else {
			// Meter must have been changed to a different
			// corridor; throw away old meter state
			meter_states.remove(meter.getName());
			return null;
		}
	}

	/** Create the meter state for a given ramp meter */
	private boolean createMeterState(RampMeterImpl meter) {
		EntranceNode en = findEntranceNode(meter);
		if (en != null) {
			MeterState ms = new MeterState(meter, en);
			meter_states.put(meter.getName(), ms);
			return true;
		} else
			return false;
	}

	/** Find an entrance node matching the given ramp meter.
	 * @param meter Ramp meter to search for.
	 * @return Entrance node matching ramp meter. */
	private EntranceNode findEntranceNode(RampMeterImpl meter) {
		R_NodeImpl rnode = findRNode(meter);
		if (null == rnode)
			return null;
		for (Node n : nodes) {
			if (n instanceof EntranceNode) {
				EntranceNode en = (EntranceNode) n;
				if (en.rnode.equals(rnode))
					return en;
			}
		}
		if (ALG_LOG.isOpen()) {
			log("Entrance " + rnode.getName() + " for " +
				meter.getName() + " not found");
		}
		return null;
	}

	/** Find node from meter onto this corridor */
	private R_NodeImpl findRNode(RampMeterImpl meter) {
		R_NodeImpl rnode = meter.getEntranceNode();
		if (null == rnode)
			return null;
		String cid = R_NodeHelper.getCorridorName(rnode);
		if (corridor.getName().equals(cid))
			return rnode;
		Corridor cor = BaseObjectImpl.corridors.getCorridor(cid);
		if (null == cor)
			return null;
		R_NodeImpl n = cor.findActiveNode(new ForkFinder(rnode));
		if (n != null)
			return n.getFork();
		else {
			log("Fork not found " + rnode.getName() +
			    " for " + meter.getName());
			return null;
		}
	}

	/** Fork finder for CD roads */
	private class ForkFinder implements Corridor.NodeFinder {
		/** Entrance node of meter on CD road */
		private final R_NodeImpl rnode;
		/** Have we found the entrance node yet? */
		private boolean found;
		/** Create a fork finder */
		private ForkFinder(R_NodeImpl n) {
			rnode = n;
		}
		public boolean check(float m, R_NodeImpl n) {
			found |= (n == rnode);
			if (!found)
				return false;
			R_NodeImpl f = n.getFork();
			return (f != null) && corridor.getName().equals(
				R_NodeHelper.getCorridorName(f)
			);
		}
	}

	/** Update the station nodes for the current interval */
	private void updateStations(long stamp) {
		for (Node n : nodes) {
			if (n instanceof StationNode) {
				StationNode sn = (StationNode) n;
				sn.updateState(stamp);
			}
		}
	}

	/** Find previous upstream station node.
	 * @return Upstream station node, or null. */
	private StationNode upstreamStation(final Node here) {
		StationNode upstream = null;
		for (Node n : nodes) {
			if (n == here)
				break;
			else if (n instanceof StationNode)
				upstream = (StationNode) n;
		}
		return upstream;
	}

	/** Find next downstream station node.
	 * @return Downstream station node, or null. */
	private StationNode downstreamStation(final Node here) {
		boolean found = false;
		for (Node n : nodes) {
			if (found && n instanceof StationNode)
				return (StationNode) n;
			if (here == n)
				found = true;
		}
		return null;
	}

	/** Is this MaxPressureAlgorithm done? */
	private boolean isDone() {
		for (MeterState ms : meter_states.values()) {
			if (ms.meter.isOperating())
				return false;
		}
		return true;
	}

	/** Node to manage station or entrance */
	abstract protected class Node {

		/** R_Node reference */
		protected final R_NodeImpl rnode;

		/** Mile point of the node */
		protected final float mile;

		/** Create a new node */
		protected Node(R_NodeImpl n, float m) {
			rnode = n;
			mile = m;
		}

		/** Get the distance to another node (in miles) */
		protected float distanceMiles(Node other) {
			return Math.abs(mile - other.mile);
		}

		/** Get the distancee to another node (in feet) */
		protected int distanceFeet(Node other) {
			return Math.round(distanceMiles(other) * FEET_PER_MILE);
		}
	}

	/** Node to manage station on corridor */
	protected class StationNode extends Node {

		/** StationImpl mapping this state */
		private final StationImpl station;
                
                // wrapper because I'm modifying SamplerSet to track cumulative counts
                private SamplerSet sampler;

		/** Density history */
		private final BoundedSampleHistory density_hist =
			new BoundedSampleHistory(steps(60));

		/** Speed history */
		private final BoundedSampleHistory speed_hist =
			new BoundedSampleHistory(steps(60));

		/** Create a new station node. */
		public StationNode(R_NodeImpl rnode, float m, StationImpl st) {
			super(rnode, m);
			station = st;
                        List<VehicleSampler> list = new ArrayList<>();
                        list.add(st);
                        sampler = new SamplerSet(list);
		}

		/** Update station state */
		private void updateState(long stamp) {
			density_hist.push(getStationDensity(stamp));
			speed_hist.push(getStationSpeed(stamp));
		}

		/** Get the current station density */
		private Double getStationDensity(long stamp) {
			float d = sampler.getDensity(stamp, PERIOD_MS);
			return (d >= 0) ? (double) d : null;
		}

		/** Get the current station speed */
		private Double getStationSpeed(long stamp) {
			float s = sampler.getSpeed(stamp, PERIOD_MS);
			return (s >= 0) ? (double) s : null;
		}

		/** Get average density of a mainline segment beginning at the
		 * current station.  This works by splitting each consecutive
		 * pair of stations into 3 equal links and assigning average
		 * density to the middle link.  All links are then averaged,
		 * weighted by length.
		 *
		 * @param dn Segment downstream station node.
		 * @return average density (distance weight). */
		private double calculateSegmentDensity(final StationNode dn) {
			StationNode cursor = this;
			double dist_seg = 0;  /* Segment distance */
			double veh_seg = 0;   /* Sum of vehicles in segment */
			double k_cursor = cursor.getDensity();
                        
                        //System.out.println("calc seg density "+cursor+" "+cursor.getDensity());
                        
			for (StationNode sn = downstreamStation(cursor);
			     sn != null && cursor != dn;
			     sn = downstreamStation(sn))
			{
                                //System.out.println("\tseg k"+sn+" "+sn.getDensity());
				double k_down = sn.getDensity();
				double k_middle = (k_cursor + k_down) / 2;
				double dist = cursor.distanceMiles(sn);
				dist_seg += dist;
				veh_seg += (k_cursor + k_middle + k_down) / 3 *
					dist;
				cursor = sn;
				k_cursor = k_down;
			}
			if (dist_seg > 0)
				return veh_seg / dist_seg;
			else
				return k_cursor;
		}

		/** Get 1 minute density at current time step.
		 * @return average 1 min density; missing data returns 0. */
		public double getDensity() {
			Double avg = density_hist.average(0, steps(60));
			return (avg != null) ? avg : 0;
		}

		/** Get 1 minute speed at current time step.
		 * @return Average 1 min speed; missing data returns 0. */
		private double getSpeed() {
			Double avg = speed_hist.average(0, steps(60));
			return (avg != null) ? avg : 0;
		}

		/** Find downstream segment station node.  This is the station
		 * downstream which results in the highest segment density.
		 * @return Downstream segment station node. */
		protected StationNode segmentStationNode() {
			StationNode dn = this;
			double dk = 0;
			for (StationNode sn = this; sn != null;
			     sn = downstreamStation(sn))
			{
				if (distanceMiles(sn) > SEGMENT_LENGTH_MILES)
					break;
				double k = calculateSegmentDensity(sn);
				if (k >= dk) {
					dk = k;
					dn = sn;
				}
			}
			return dn;
		}

		/** Get a string representation of a station node */
		@Override
		public String toString() {
			return "SN:" + station.getName();
		}
	}

	/** Node to manage entrance onto corridor */
	class EntranceNode extends Node {

		/** Create a new entrance node */
		public EntranceNode(R_NodeImpl rnode, float m) {
			super(rnode, m);
		}

		/** Get a string representation of an entrance node */
		@Override
		public String toString() {
			return "EN:" + rnode.getName();
		}
	}

	/** Enum for metering phase */
	private enum MeteringPhase {
		not_started,
		metering,
		flushing,
		stopped,
	};

	/** Ramp meter state */
	class MeterState {

		/** Meter at this entrance */
		private final RampMeterImpl meter;

		/** Entrance node for the meter */
		private final EntranceNode node;

		/** Station node association */
		private final StationNode s_node;

		/** Queue sampler set */
		private final CCSamplerSet queue;

		/** Passage sampler set */
		private final CCSamplerSet passage;

		/** Merge sampler set */
		private final CCSamplerSet merge;

		/** Bypass sampler set */
		private final CCSamplerSet bypass;

		/** Green count sampler set */
		private final CCSamplerSet green;

		/** Metering phase */
		private MeteringPhase phase = MeteringPhase.not_started;

		/** Is the meter currently metering? */
		private boolean isMetering() {
			return phase != MeteringPhase.not_started &&
			       phase != MeteringPhase.stopped;
		}

		/** Minimum metering rate (vehicles / hour) */
		private int min_rate = 0;

		/** Current metering rate (vehicles / hour) */
            private int release_rate = 0;

		/** Maximum metering rate (vehicles / hour) */
		private int max_rate = 0;

		

		/** Cumulative demand count (vehicles) */
		private float demand_accum = 0;

		/** Demand adjustment (vehicles) */
		private float demand_adj = 0;

		/** Tracking queue demand rate (vehicles / hour) */
		private int tracking_demand = 0;

		/** Passage sampling good (latches until queue empty) */
		private boolean passage_good = true;

		/** Cumulative passage count (vehicles) */
		private int passage_accum = 0;

		

		/** Cumulative green count (vehicles) */
		private int green_accum = 0;

		/** Time queue has been empty (seconds) */
		private int queue_empty_secs = 0;

		/** Time queue has been backed-up (seconds) */
		private int queue_backup_secs = 0;

		/** Total occupancy for duration of a queue backup */
		private int backup_occ = 0;

		/** Controlling minimum rate limit */
		private MinimumRateLimit limit_control =
			MinimumRateLimit.target_min;

		/** End time stamp */
		private long stamp;
                
                
                // these are capacities for mainline and ramp
                private double Q_u = 0;
                private double Q_r = 0;
                private double Q_d = 0;
                // jam densities
                private double K_d = 0;
                private double K_u = 0;
                private double K_r = 0;
                // free flow speeds
                private double v_u = 0;
                private double v_d = 0;
                private double v_r = 0;
                // backwards wave speeds
                private double w_u = 0;
                private double w_d = 0;
                private double w_r = 0;
                // critical densities
                private double kc_u = 0;
                private double kc_d = 0;
                private double kc_r = 0;
                // length
                private double L_u = 0;
                private double L_d = 0;
                
                // upstream node, node at merge point, downstream, node for mainline
                private StationNode upstream;
                private StationNode downstream;
                private StationNode mergepoint; 
                
                private CCSamplerSet N_u;
                private CCSamplerSet N_d;
                private CCSamplerSet N_mid;
                
                private double ramp_length = 0;
                
                



		/** Create a new meter state */
		public MeterState(RampMeterImpl mtr, EntranceNode en) {
			meter = mtr;
			node = en;
                        
                        //System.out.println("meter state inst "+queue.size()+" "+passage.size()+" "+merge.size());
			s_node = getAssociatedStation();
                        
                        
                        // I need the free flow speed
                        v_u = s_node.rnode.getSpeedLimit() + 5; // or maybe 75?
                        v_r = 45; // ramp free flow speed
                        
                        // values from SUMO
                        v_u = 60;
                        
                        v_d = v_u;
                        
                        double min_link_len = v_u * STEP_SECONDS/3600;
                        
                        mergepoint = findMergePoint(s_node); // this is not always the same as s_node
                        upstream = findUpstreamStation(s_node, min_link_len);
                        downstream = findDownstreamStation(s_node, min_link_len);
                        
                        
                        
                        int ramp_lanes = 2; // assume 2 lanes form on ramp
                        int upstream_lanes = upstream.rnode.getLanes(); // number of lanes on upstream. Used for capacity, etc.
                        
                        
                        K_u = upstream_lanes * K;
                        K_r = ramp_lanes * K;
                        K_d = upstream_lanes * K;
                        
                        
                        // capacity
                        // adjust these for ACC
                        Q_u = Math.min(2400, 2200 + 10 * (v_u - 50)) * upstream_lanes; // from HCM
                        Q_r = 1900; // from HCM, base ramp saturation flow
                        
                        
                        
                        Q_u = 2150 * upstream_lanes;
                        Q_r = 2107;
                        
                        
                        Q_d = Q_u;
                        
                        L_u = mergepoint.mile - upstream.mile;
                        L_d = downstream.mile - mergepoint.mile;
    
                        
                        // calculate critical density. Assume triangle.
                        kc_u = Q_u / v_u;
                        kc_r = Q_r / v_r;
                        kc_d = kc_u;
                        
                        // backwards wave speed
                        w_u = Q_u / (K_u - kc_u);
                        w_r = Q_r / (K_r - kc_r);
                        w_d = w_u;
                        
                        

                        System.out.println("check rate bounds "+RampMeterHelper.getMaxRelease()+" "+RampMeterHelper.getMinRelease());
                        
                        
                        // this has units of hours; convert to ms
                        
                        
                        stamp = DetectorImpl.calculateEndTime(PERIOD_MS);
                        
                        long max_lookback_r = STEP_SECONDS * 10 * 1000;
                        
			queue = new CCSamplerSet(meter.getSamplerSet(LaneCode.QUEUE), max_lookback_r, stamp);
			passage = new CCSamplerSet(meter.getSamplerSet(LaneCode.PASSAGE), max_lookback_r, stamp);
			merge = new CCSamplerSet(meter.getSamplerSet(LaneCode.MERGE), max_lookback_r, stamp);
			bypass = new CCSamplerSet(meter.getSamplerSet(LaneCode.BYPASS), max_lookback_r, stamp);
			green = new CCSamplerSet(meter.getSamplerSet(LaneCode.GREEN), max_lookback_r, stamp);
                        
                        
                        long max_lookback_us = STEP_SECONDS * 2 * 1000 + (long)Math.ceil(L_u / Math.min(v_u, w_u) * 3600 * 1000);
                        N_u = new CCSamplerSet(upstream.rnode.getSamplerSet(), max_lookback_us, stamp);
                        
                        long max_lookback_ds = STEP_SECONDS * 2 * 1000 + (long)Math.ceil(L_d / Math.min(v_d, w_d) * 3600 * 1000);
                        N_d = new CCSamplerSet(downstream.rnode.getSamplerSet(), max_lookback_ds, stamp);
                        N_mid = new CCSamplerSet(mergepoint.rnode.getSamplerSet(), Math.max(max_lookback_us, max_lookback_ds), stamp);
                        
                        
                        
		}
                
                private StationNode findMergePoint(Node start){
                    
                    StationNode closest = null;
                    for(Node n : nodes){
                        if(n instanceof StationNode)
                        System.out.println("\t"+n+" "+n.mile+" "+node+" "+node.mile);
                    }
                    
                    for(Node n : nodes){
                        if(start.mile > n.mile){
                            if(n instanceof StationNode){
                                closest = (StationNode)n;
                            }
                        }
                        else{
                            // we've gone too far
                            break;
                        }
                    }
                    
                    return s_node;
                }
                
                
                private StationNode findUpstreamStation(Node start, double min_link_len){
                    // assume that nodes are in sorted order
                    // s_node is the base point. I want the closest node with distance > min_link_len
                    StationNode closest = null;
                    
                    for(Node n : nodes){
                        if(start.mile - n.mile > min_link_len){
                            if(n instanceof StationNode){
                                closest = (StationNode)n;
                            }
                        }
                        else{
                            // we've gone too far
                            break;
                        }
                    }
                    
                    return closest;
                }
                
                private StationNode findDownstreamStation(Node start, double min_link_len){
                    // assume nodes in sorted order
                    // s_node is the base point. I want the closest node with distance > min_link_len
  
                    for(Node n : nodes){
                        // first matching node is the closest
                        if((n instanceof StationNode) && n.mile - start.mile  > min_link_len){
                            return (StationNode)n;
                        }
                    }
                    
                    //System.out.println(closest);
                    
                    return null;
                }

		/** Get station to associate with the meter state.
		 * @return Associated station node, or null. */
		private StationNode getAssociatedStation() {
			StationNode us = getAssociatedUpstream();
			StationNode ds = downstreamStation(node);
			return useDownstream(us, ds) ? ds : us;
		}

		/** Get associated upstream station.
		 * @return Station node upstream of meter, or null. */
		private StationNode getAssociatedUpstream() {
			StationNode us = upstreamStation(node);
			return isUpstreamStationOk(us) ? us : null;
		}

		/** Check if an upstream station is OK.
		 * @param us Station just upstream of meter.
		 * @return true if upstream station is suitable. */
		private boolean isUpstreamStationOk(StationNode us) {
			return us != null &&
			       node.distanceMiles(us) < UPSTREAM_STATION_MILES;
		}

		/** Test if downstream station should be associated.
		 * @param us Station just upstream of meter.
		 * @param ds Station just downstream of meter.
		 * @return true if downstream station should be associated. */
		private boolean useDownstream(StationNode us, StationNode ds) {
			if (us == null)
				return true;
			if (ds == null)
				return false;
			int uf = node.distanceFeet(us);
			int df = node.distanceFeet(ds);
			return df < DOWNSTREAM_STATION_FEET && df < uf;
		}



		/** Validate meter state.
		 *   - Update state timers.
		 *   - Update passage flow and accumulator.
		 *   - Update demand flow and accumulator.
		 *   - Calculate metering rate. */
		private void validate() {
                    
                    stamp = DetectorImpl.calculateEndTime(PERIOD_MS);
                    
     
			min_rate = filterRate(calculateMinimumRate(stamp));
			max_rate = filterRate(calculateMaximumRate());
                        
                        System.out.println("validate "+meter.getName()+" "+s_node+" "+min_rate+" "+max_rate);
			if (s_node != null)
                            calculateMeteringRate();
		}

		/** Check the queue backed-up state */
		private void checkQueueBackedUp() {
			if (isQueueOccupancyHigh()) {
				queue_backup_secs += STEP_SECONDS;
				backup_occ += queue.getMaxOccupancy(stamp,
					PERIOD_MS);
			} else {
				queue_backup_secs = 0;
				backup_occ = 0;
			}
		}

		/** Check if queue is empty */
		private void checkQueueEmpty() {
			if (isQueuePossiblyEmpty())
				queue_empty_secs += STEP_SECONDS;
			else
				queue_empty_secs = 0;
			// Get rid of unused greens
			if (queue_empty_secs > QUEUE_EMPTY_RESET_SECS &&
			    isPassageBelowGreen())
				green_accum = passage_accum;
		}


		/** Calculate passage count (vehicles).
		 * @return Passage vehicle count */
		private int calculatePassageCount() {
			int vol = passage.getVehCount(stamp, PERIOD_MS);
			if (vol >= 0)
				return vol;
			vol = merge.getVehCount(stamp, PERIOD_MS);
			if (vol >= 0) {
				int b = bypass.getVehCount(stamp, PERIOD_MS);
				if (b > 0) {
					vol -= b;
					if (vol < 0)
						return 0;
				}
				return vol;
			}
			return MISSING_DATA;
		}

	

		/** Get queue demand count for the current period */
		private float queueDemandCount() {
			float vol = queue.getVehCount(stamp, PERIOD_MS);
			if (vol >= 0)
				return vol;
			else {
				int target = getDefaultTarget();
				return vehCountPeriod(target, STEP_SECONDS);
			}
		}

		/** Calculate the demand adjustment.
		 * @return Demand adjustment (number of vehicles) */
		private float calculateDemandAdjustment() {
			return estimateDemandUndercount()
			     - estimateDemandOvercount();
		}

		/** Estimate demand undercount when occupancy is high.
		 * @return Demand undercount (may be negative). */
		private float estimateDemandUndercount() {
			return queueFullRatio() * availableStorage();
		}

		/** Estimate the queue full ratio.
		 * @return Ratio from 0 to 1. */
		private float queueFullRatio() {
			float qor = queueOccRatio();
			float qbr = queueRatio(queue_backup_secs);
			return Math.max(qor, qbr);
		}

		/** Get queue occupancy ratio.  Map occupancy values between
		 * QUEUE_OCC_THRESHOLD and 100% to a range of 0 and 1.
		 * @return Ratio from 0 to 1. */
		private float queueOccRatio() {
			float o = queue.getMaxOccupancy(stamp, PERIOD_MS)
				- QUEUE_OCC_THRESHOLD;
			return (o > 0)
			     ? Math.min(o / (100 - QUEUE_OCC_THRESHOLD), 1)
			     : 0;
		}

		/** Calculate a queue ratio.
		 * @param secs Number of seconds.
		 * @return Ratio compared to max wait time, between 0 and 1. */
		private float queueRatio(int secs) {
			return Math.min(2 * secs / maxWaitTime(), 1);
		}

		/** Estimate the available storage in queue.
		 * @return Available storage (vehicles, may be negative). */
		private float availableStorage() {
			if (passage_good) {
				float q_len = Math.max(queueLength(), 0);
				return maxStorage() - q_len;
			} else
				return maxStorage() / 3;
		}

		/** Estimate demand overcount when queue is empty.
		 * @return Vehicle overcount at queue detector (may be
		 *         negative). */
		private float estimateDemandOvercount() {
			return queueRatio(queue_empty_secs) * queueLength();
		}

		/** Estimate the length of queue (vehicles).
		 * @return Queue length (may be negative). */
		private float queueLength() {
                    long stamp = DetectorImpl.calculateEndTime(PERIOD_MS);
                    return (float)getRampQueueLength(stamp);
		}



		/** Get the default target metering rate (vehicles / hour) */
		private int getDefaultTarget() {
			int t = meter.getTarget();
			return (t > 0) ? t : getMaxRelease();
		}

		/** Check if the queue is possibly empty */
		private boolean isQueuePossiblyEmpty() {
			return isQueueFlowLow() && !isQueueOccupancyHigh();
		}

		/** Check if the queue flow is low.  If the passage detector
		 * is not good, assume this is false. */
		private boolean isQueueFlowLow() {
			return (passage_good)
			     ? (isDemandBelowPassage() || isPassageBelowGreen())
			     : false;
		}

		/** Check if cumulative demand is below cumulative passage */
		private boolean isDemandBelowPassage() {
			return queueLength() < QUEUE_EMPTY_THRESHOLD;
		}

		/** Check if queue occupancy is above threshold */
		private boolean isQueueOccupancyHigh() {
			float occ = queue.getMaxOccupancy(stamp, PERIOD_MS);
			return occ > QUEUE_OCC_THRESHOLD;
		}

		/** Check if cumulative passage is below cumulative green */
		private boolean isPassageBelowGreen() {
			return violationCount() < QUEUE_EMPTY_THRESHOLD;
		}

		/** Calculate violation count (passage above green count) */
		private int violationCount() {
			return (passage_good)
			     ? (passage_accum - green_accum)
			     : 0;
		}

		/** Get meter queue state enum value */
		private MeterQueueState getQueueState() {
			if (isMetering()) {
				if (isQueueFull())
					return MeterQueueState.FULL;
				else if (!passage_good)
					return MeterQueueState.UNKNOWN;
				else if (isQueueEmpty())
					return MeterQueueState.EMPTY;
				else
					return MeterQueueState.EXISTS;
			}
			return MeterQueueState.UNKNOWN;
		}

		/** Check if the ramp meter queue is full */
		private boolean isQueueFull() {
			return isQueueOccupancyHigh() ||
			      (isQueueLimitFull() && !isPassageBelowGreen());
		}

		/** Check if the meter queue is full (by storage/wait limit) */
		private boolean isQueueLimitFull() {
			return queue.isPerfect() &&
			       passage_good &&
			      (isQueueStorageFull());
		}

		/** Check if the ramp queue storage is full */
		private boolean isQueueStorageFull() {
			return queueLength() >= targetStorage();
		}

		

		/** Check if the meter queue is empty */
		private boolean isQueueEmpty() {
			return queueLength() < 1;
		}

		/** Calculate minimum rate (vehicles / hour) */
		private int calculateMinimumRate(long stamp) {
                        
                    min_rate = 360;

                    double ramp_queue = getRampQueueLength(stamp);
                    // if queue is full
                    double ramp_length = 0.25;
                    if(ramp_queue > ramp_length * K_r * 0.9){ // 90% of jam density
                        min_rate = getMaximumRate();
                    }
                    /*
                    else{ // rate for 4 min waiting time
                        min_rate = (int)Math.min(max_rate, Math.max(360, (int)Math.round(ramp_queue / 4.0 * 60))); // rate for 4 min waiting time
                    }
                    */
                    
                    
                    return min_rate;
		}

		

		/** Caculate queue storage limit.  Project into the future the
		 * duration of the target wait time.  Using the target demand,
		 * estimate the cumulative demand at that point in time.  From
		 * there, subtract the target ramp storage count to find the
		 * required cumulative passage vehicle count at that time.
		 * @return Queue storage limit (vehicles / hour). */
		private int queueStorageLimit() {
			assert passage_good;
			float proj_arrive = vehCountPeriod(tracking_demand,
				targetWaitTime());
			float demand_proj = demand_accum + proj_arrive;
			int req = Math.round(demand_proj - targetStorage());
			int pass_min = req - passage_accum;
                        
                        //System.out.println("qsl "+proj_arrive+" "+demand_proj+" "+req+" "+flowRate(pass_min, steps(targetWaitTime())));
			return flowRate(pass_min, steps(targetWaitTime()));
		}

		/** Calculate the target storage on the ramp (vehicles) */
		private float targetStorage() {
			return maxStorage() * STORAGE_TARGET_RATIO;
		}

		/** Calculate the maximum storage on the ramp (vehicles) */
		private float maxStorage() {
			int stor_ft = meter.getStorage() * meter.getLaneCount();
			return stor_ft * JAM_VPF;
		}

	

		/** Get the target wait time (seconds) */
		private int targetWaitTime() {
			return Math.round(maxWaitTime() * WAIT_TARGET_RATIO);
		}

		/** Get the max wait time (seconds) */
		private float maxWaitTime() {
			return Math.max(meter.getMaxWait(), 1);
		}

		

		/** Calculate target minimum rate.
		 * @return Target minimum rate (vehicles / hour). */
		private int targetMinRate() {
			return Math.round(tracking_demand * TARGET_MIN_RATIO);
		}

		/** Calculate backup minimum limit.
		 * @return Backup minimum limit (vehicles / hour). */
		private int backupMinLimit() {
			// NOTE: The following is the proper calculation:
			//    occ_avg = backup_occ / steps(queue_backup_secs)
			//    backup_mins = queue_backup_secs / 60.0f
			//    ratio = 50 + occ_avg * backup_mins
			// This can be simplified to this:
			//    ratio = 50 + backup_occ * STEP_SECONDS *
			//        queue_backup_secs / (queue_backup_secs * 60)
			// Which can be further simplified to:
			//    ratio = 50 + backup_occ * STEP_SECONDS / 60
			float ratio = BACKUP_LIMIT_BASE + backup_occ *
				STEP_SECONDS / (60.0f * 100.0f);
			return Math.round(tracking_demand * ratio);
		}

		/** Get the target maximum rate ratio for current phase */
		private float targetMaxRatio() {
			switch (phase) {
			case flushing:
			     return TARGET_MAX_RATIO_FLUSHING;
			default:
			     return TARGET_MAX_RATIO;
			}
		}

		/** Calculate target maximum rate.
		 * @return Target maxumum rate (vehicles / hour). */
		private int calculateMaximumRate() {
                    return RampMeterHelper.getMaxRelease();
		}

		/** Calculate the metering rate */
		private void calculateMeteringRate() {

                        
                        System.out.println("calc meter rate "+meter.getName());
                        
            
                        long stamp = DetectorImpl.calculateEndTime(PERIOD_MS);
                        
                        
                        
                        
                        
                        // need num lanes to help calculate capacities
                        int numlanes = upstream.rnode.getLanes();
                        
                        // I need sending flow, receiving flow for mainline and ramp
                        double S_rd = getRampSendingFlow(stamp); 
                        double S_ud = getUpstreamSendingFlow(stamp);
                        double R_d = getDownstreamReceivingFlow(stamp);
                        
                        
                        queue.log(stamp, PERIOD_MS);
                        green.log(stamp, PERIOD_MS);
                        
                        System.out.println("S_ud "+S_ud+" S_rd "+S_rd+" R_d "+R_d);
                        
                        
                        // these are weighting factors
                        double c_u = 1;
                        double c_r = numlanes / 2.0;
                        double c_d = 1;
                        
                        // these are the position weights
                        double downstream_weight = c_d * getDownstreamWeight(stamp);
                        double ramp_weight = c_r * getRampWeight(stamp);
                        double upstream_weight = c_u * getUpstreamWeight(stamp);
                        
                        System.out.println(downstream_weight+" "+ramp_weight+" "+upstream_weight);
                        double weight_ud = upstream_weight - downstream_weight;
                        double weight_rd = ramp_weight - downstream_weight;
                        
                        System.out.println(weight_ud +" "+weight_rd);
                        
                        
                        
                        int max_rate = getMaximumRate();
                        int min_rate = Math.min(max_rate, getMinimumRate()); // if min rate is less than max rate for some reason, use max rate
                        
                        
                        
                        
                        

                        System.out.println("rate min "+min_rate+" max "+max_rate);
                        
                        int new_rate = calcBestRate(S_ud, S_rd, R_d, weight_ud, weight_rd, min_rate, max_rate);
    
                        // if equal to capacity, that is effectively no metering
                        // null means meter is off
                        if(new_rate >= Math.floor(Q_r)){
                            meter.setRatePlanned(null);
                            release_rate = (int)Q_r;
                            
                        }
                        else{
                            meter.setRatePlanned(new_rate);
                            release_rate = new_rate;
                        }
                        
		}
                
                private int calcBestRate(double S_ud, double S_rd, double R_d, double weight_ud, double weight_rd, int min_rate, int max_rate){
                    
                    // ignore negative values from detector errors
                    S_ud = Math.max(0, S_ud);
                    S_rd = Math.max(0, S_rd);
                    
                    R_d = Math.max(0, R_d);
                    
                    // base case: no metering
                    int best_rate = (int)Q_r;
                    
                    
                    // S <= R: everyone can move.
                    if(S_ud + S_rd <= R_d){
                        return best_rate;
                    }
                    
                    // check obj value for no metering
                    double best_obj = calcMPObj(weight_ud, weight_rd, S_ud, S_rd, R_d, Q_u, Q_r);

                    // brute force line search
                    
                    
                    
                    //System.out.println("min "+min_rate+" "+max_rate);
                    int interval_rate = 10;

                    // what do I do about maximum rate? I should also consider meter off as an option. 

                    

                    for(int rate = min_rate; rate <= max_rate; rate+= interval_rate){

                        double obj = calcMPObj(weight_ud, weight_rd, S_ud, Math.min(S_rd, rate * STEP_SECONDS/3600.0), R_d, Q_u, rate); // this uses the rate as the capacity for the ramp

                        if(obj >= best_obj){
                            best_rate = rate;
                            best_obj = obj;
                        }
                    }   
                    
                    System.out.println("calc best rate "+best_rate+" "+best_obj);
                    
                    return best_rate;
                }
                
                
                private double calcMPObj(double w_ud, double w_rd, double S_ud, double S_rd, double R_d, double Q_u, double Q_r){
                    double y_ud = 0;
                    double y_rd = 0;
                    
                    // uncongested merge case
                    if(S_ud + S_rd < R_d){
                        y_ud = S_ud;
                        y_rd = S_rd;
                    }
                    // congested merge case, 2 upstream links
                    else{
                        double lambda_rd = R_d * Q_r / (Q_u + Q_r);
                        y_rd = median(R_d - S_ud, S_rd, lambda_rd);
                        y_ud = R_d - y_rd;
                    }
                    
                    return w_ud * y_ud + w_rd * y_rd;
                }
                
                private double getRampQueueLength(long stamp){
                    return queue.getCumulativeCount(stamp, PERIOD_MS) - green.getCumulativeCount(stamp, PERIOD_MS);
                }
                private double getRampSendingFlow(long stamp){
                    double S_rd = Math.min(Q_r * STEP_SECONDS / 3600.0, getRampQueueLength(stamp));
                    
                    if(S_rd < 0){
                        queue.log(stamp, PERIOD_MS, System.err);
                        green.log(stamp, PERIOD_MS, System.err);
                        throw new RuntimeException("ramp queue is negative");
                    }
                    
                    return S_rd;
                }
                
                private double getUpstreamSendingFlow(long stamp){
                    long lookbehind = (long)Math.round(1000*(STEP_SECONDS - L_u/v_u*3600));
                   

                    double N_up = N_u.getCumulativeCount(stamp + lookbehind, PERIOD_MS);
                    double N_down = N_mid.getCumulativeCount(stamp, PERIOD_MS);
                    double S_ud = Math.min(Q_u * STEP_SECONDS / 3600.0, N_up - N_down);
                    return S_ud;
                }
                
                private double getDownstreamReceivingFlow(long stamp){
                    long lookbehind = (long)Math.round(1000*(STEP_SECONDS - L_d/w_d*3600));
                    double N_down = N_d.getCumulativeCount(stamp + lookbehind, PERIOD_MS);
                    double N_up = N_mid.getCumulativeCount(stamp, PERIOD_MS) + merge.getCumulativeCount(stamp, PERIOD_MS);
                    double R_d = Math.min(Q_d * STEP_SECONDS / 3600.0, N_up - N_down + K_u * L_d);
                    
                    //System.out.println("check R_d "+N_down+" "+N_up+" "+K_u+" "+L_d);
                    return R_d;
                }
                
                private double getUpstreamWeight(long stamp){
                    
                    double[] N = new double[NUM_PRESSURE_INTERVAL+1];
                    
                    double link_len = v_u * STEP_SECONDS /3600.0; // if the link is very long, don't use the entire thing!
                    
                    N[N.length-1] = N_mid.getCumulativeCount(stamp, PERIOD_MS);
                    
                    double interval_length = link_len/NUM_PRESSURE_INTERVAL;
                    
             
                    
                    for(int idx = 0; idx < N.length-1; idx++){
                        // use newell's method
                        double L_congested = interval_length * (N.length-1-idx);
                        double L_uncongested = L_u - L_congested;
                        
                        long lookbehind_u = (long)Math.round(L_uncongested / v_u * 3600 * 1000);
                        
                        double N_uncongested = N_u.getCumulativeCount(stamp - lookbehind_u, PERIOD_MS); // in ms
                        
                        long lookbehind_d = (long)Math.round(L_congested / w_u * 3600 * 1000);
                        double N_congested = L_congested * K_u + N_mid.getCumulativeCount(stamp - lookbehind_d , PERIOD_MS); // in ms;
                        N[idx] = Math.min(N_uncongested, N_congested);
        
                        //System.out.println("\tcheck N calc "+N_uncongested+" "+N_congested+" "+L_uncongested+" "+L_congested+" "+lookbehind_u+" "+(stamp - lookbehind_u));
                    }
                    
                    //System.out.println("check upstream N time "+stamp+" "+Arrays.toString(N));
                    //N_u.log(stamp, PERIOD_MS);
                    
 
                    
                    double total = 0;
                    
                    //System.out.println("check calc "+N[0]+" "+N[N.length-1]);
                    for(int idx = 0; idx < N.length - 1; idx++){
                        double density = Math.max(0, (N[idx] - N[idx+1]) / interval_length); // density might be less than 0 due to sensor error; do not admit negative density
                        // I want to integrate density * x/L over the interval len*idx, len*(idx+1)
                        // integral of x is x^2
                        double start = interval_length * idx;
                        double end = interval_length * (idx+1);
                        
                        
                        double integral = (end * end/2 - start * start/2 ) * density / link_len;
                        
                        //System.out.println("\tcheck calc "+end+" "+integral+" "+density);
                        total += integral;
                    }
                    
                    return total;
                }
                
                private double getRampWeight(long stamp){
                    //double assumed_length = v_r * STEP_SECONDS / 3600.0; // calculate assumed ramp length for purposes of matching the units of upstream and downstream weights
                    double assumed_length = v_u * STEP_SECONDS /3600.0; // match upstream weight
                    double ramp_length = 0.25; // this is hardcoded!
                    double density = (queue.getCumulativeCount(stamp, PERIOD_MS) - passage.getCumulativeCount(stamp, PERIOD_MS)) / ramp_length;
                    double integral = assumed_length / 2 * density; // this assumes uniform density...probably not correct...
                    
                    
                    
                    // for the ramp pressure, it is simply the number of vehicles waiting
                    return integral;
                }
                
                private double getDownstreamDensity(long stamp){
                    return (N_d.getCumulativeCount(stamp, PERIOD_MS) - (N_mid.getCumulativeCount(stamp, PERIOD_MS) + merge.getCumulativeCount(stamp, PERIOD_MS))) / L_d;
                }
                
                private double getDownstreamWeight(long stamp){
                    
                    double link_len = v_d * STEP_SECONDS / 3600; // this may not go all the way to the next loop detector
                    
                    double[] N = new double[NUM_PRESSURE_INTERVAL+1];
                    N[0] = N_mid.getCumulativeCount(stamp, PERIOD_MS) + merge.getCumulativeCount(stamp, PERIOD_MS);
                    
                    
                    double interval_length = link_len/NUM_PRESSURE_INTERVAL;
                    
                    for(int idx = 1; idx < N.length; idx++){
                        // use newell's method
                        double L_uncongested = interval_length * idx;
                        double L_congested = L_d - L_uncongested;
                        long lookbehind_u = (long)Math.round(L_uncongested / v_d * 3600 * 1000);
                        double N_uncongested = N_mid.getCumulativeCount(stamp - lookbehind_u, PERIOD_MS) + merge.getCumulativeCount(stamp - lookbehind_u, PERIOD_MS); // in ms
                        long lookbehind_d = (long)Math.round(L_congested / w_d * 3600 * 1000);
                        double N_congested = L_congested * K_d + N_d.getCumulativeCount(stamp - lookbehind_d, PERIOD_MS); // in ms;
                        N[idx] = Math.min(N_uncongested, N_congested);
                        
                        //System.out.println("\tcheck N "+N[idx]+" "+N_uncongested+" "+N_congested+" "+lookbehind_u+" "+(stamp-lookbehind_u)+" "+lookbehind_d+" "+(stamp-lookbehind_d));
                    }
                    
                    double total = 0;
                    
                    //N_d.log(stamp, PERIOD_MS);
                    //System.out.println("check N "+Arrays.toString(N));
                    
                    for(int idx = 0; idx < N.length - 1; idx++){
                        double density = Math.max(0, (N[idx] - N[idx+1]) / interval_length); // density might be less than 0 due to sensor error; do not admit negative density
                        // I want to integrate density * (L-x)/L over the interval len*idx, len*(idx+1)
                        // integral of x is x^2. Integral of constant is x.
                        double start = interval_length * idx;
                        double end = interval_length * (idx+1);
                        double integral = ((end - start) - (end*end/2 - start*start/2) / link_len) * density;
                        
                        
                        
                        total += integral;
                    }
                    
                    return total;
                }
                
                private double median(double a, double b, double c){
                    return Math.max(Math.min(a,b), Math.min(Math.max(a,b),c));
                }

		

		
	

		/** Get the minimum metering rate.
		 * @return Minimum metering rate */
		private int getMinimumRate() {
			return min_rate;
		}

		/** Get the maximum metering rate.
		 * @return Maximum metering rate */
		private int getMaximumRate() {
			return max_rate;
		}

		/** Limit metering rate within minimum and maximum rates */
		private double limitRate(double r) {
			return Math.min(getMaximumRate(),
			       Math.max(getMinimumRate(), r));
		}

		/** Log a meter event */
		protected void logMeterEvent() {
			long stamp = DetectorImpl.calculateEndTime(PERIOD_MS);
			Double sd = getDownstreamDensity(stamp);
                        String dns = meter.getName();
			float seg_den = (sd != null) ? sd.floatValue() : 0;
			MeterEvent ev = new MeterEvent(EventType.METER_EVENT,
				meter.name, phase.ordinal(),
				getQueueState().ordinal(), queueLength(),
				demand_adj, estimateWaitSecs(),
				limit_control.ordinal(), min_rate, release_rate,
				max_rate, dns, seg_den);
			BaseObjectImpl.logEvent(ev);
		}
                
                private int estimateWaitSecs(){
                    return (int)Math.round(queueLength() * 3600.0 / release_rate); 
                }

		/** Get current metering rate.
		 * @return metering rate */
		private double getRate() {
			double r = release_rate;
                        
                        return r;
		}

		

		/** Get the downstream node for the segment */
		private StationNode segmentDownstream() {
			if (s_node != null) {
				StationNode dn = s_node.segmentStationNode();
				if (dn != null)
					return dn;
			}
			return null;
		}

		/** Get a string representation of a meter state */
		@Override
		public String toString() {
			return "meter:" + meter.getName() + " rate: "+release_rate;
		}
	}
}

using JLD
using PyPlot

# Access Python modules for path processing.  Ugly way of doing it, can seek to clean this up in the future.
using PyCall
const path_utils_loc = "/home/govvijay/catkin_ws/src/mkz_mpc_path_follower/scripts/path_utils"
unshift!(PyVector(pyimport("sys")["path"]), path_utils_loc) # append the current directory to Python path
@pyimport nav_msgs_path_frenet as nmp
@pyimport numpy as py_np

function compare_timestamp(ts1::ASCIIString, ts2::ASCIIString)
	val1 = parse(Int, split(ts1, "_")[2])
	val2 = parse(Int, split(ts2, "_")[2])
	return val1 < val2
end

function parse_dict_entry(entry)
	    ros_tm = entry[1]

		s_mpc   = entry[2][1]
		ey_mpc   = entry[2][2]
		v_mpc   = entry[2][3]
		epsi_mpc = entry[2][4]

		K   = entry[2][5]
		path_ref = entry[2][6]

		# Optimal Solution
        d_f_opt = entry[2][7]
        acc_opt = entry[2][8]

		return ros_tm, s_mpc, ey_mpc, v_mpc, epsi_mpc, K, path_ref, d_f_opt, acc_opt
end

function plot_mpc_trajs(fname, logdir)
	data_dict = load(@sprintf("%s/%s", logdir, fname))

	f = figure(figsize=(8.4,8.8))	
	plt[:ion]()
	for key in sort(collect(keys(data_dict)), lt=(x,y)->compare_timestamp(x,y))

		tm, sm, eym, vm, epm, K, pr, df, acc = parse_dict_entry(data_dict[key])
		v_ref = 15.0*ones(length(vm))

		x_arr = pr["x"]
		y_arr = pr["y"]
		s_max = pr["s"][end]

		xrecon, yrecon, precon, srecon = nmp.reconstruct_frenet(x_arr[1], y_arr[1], -epm[1], s_max, K)

		x_mpc = py_np.interp(sm, srecon, xrecon)
		y_mpc = py_np.interp(


		
		plot(x_arr, y_arr, color="k", marker = "x", label="REF")
		plot(xrecon, yrecon, color="b", label="RECON")
		xlabel("X (m)")
	    ylabel("Y (m)")
	    show()
		plt[:pause](0.01)
		plt[:clf]()

		#=
		# plot the results
    	subplot(711)
		plot(xm, ym, color="r", marker = "o", label="MPC")
		plot(xr, yr, color="k", label="Ref")
		#legend()
	    xlabel("X (m)")
	    ylabel("Y (m)")

		subplot(712)
    	plot(t_ref, xm, color="r", marker="o", label="MPC")
    	plot(t_ref, xr, color="k", label="Desired")
    	#xlabel("Time (s)")
    	ylabel("X (m)")
 
	    subplot(713)
    	plot(t_ref, ym, color="r", marker="o", label="MPC")
    	plot(t_ref, yr, color="k", label="Desired")
    	#xlabel("Time (s)")
    	ylabel("Y (m)")

	    subplot(714)
    	plot(t_ref, pm, color="r", marker="o", label="MPC")
    	plot(t_ref, pr, color="k", label="Desired")
    	#xlabel("Time (s)")
    	ylabel("Psi (rad)")

	    subplot(715)
    	plot(t_ref, vm, color="r", marker="o", label="MPC")
    	plot(t_ref, v_ref, color="k", label="Desired")
    	#xlabel("Time (s)")
    	ylabel("V (m/s)")

	    subplot(716)
    	plot(t_ref[1:end-1], acc, color="r", marker="o", label="MPC")
    	#xlabel("Time (s)")
    	ylabel("A (m/s^2)")

	    subplot(717)
    	plot(t_ref[1:end-1], df, color="r", marker="o", label="MPC")
    	#xlabel("Time (s)")
    	ylabel("Steering Angle (rad)")

	    #tight_layout()
		subplots_adjust(left=0.12, bottom=0.07, right=0.97, top=0.95, wspace=0.1, hspace=0.3)
    	suptitle(@sprintf("Ros Time: %.3f", tm))
		#show()
		fig_num = parse(Int, split(key, "_")[2])
		fig_str = lpad(fig_num, 4, 0)
    	savefig(@sprintf("%s/figs/%s_mpc.png", logdir, fig_str))
		close(f)
		=#
	end
end


if ! isinteractive()
	
	logdir = "/home/govvijay/Desktop/frenet_example"
	fname = "working_ish.jld"
	plot_mpc_trajs(fname, logdir)
end

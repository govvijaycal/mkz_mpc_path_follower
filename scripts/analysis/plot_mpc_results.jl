using JLD
using PyPlot

function compare_timestamp(ts1::ASCIIString, ts2::ASCIIString)
	val1 = parse(Int, split(ts1, "_")[2])
	val2 = parse(Int, split(ts2, "_")[2])
	return val1 < val2
end

function parse_dict_entry(entry)
	    ros_tm = entry[1]

		x_mpc   = entry[2][1]
		y_mpc   = entry[2][2]
		v_mpc   = entry[2][3]
		psi_mpc = entry[2][4]

		x_ref   = entry[2][5]
		y_ref   = entry[2][6]
		psi_ref = entry[2][7]

		# Optimal Solution
        d_f_opt = entry[2][8]
        acc_opt = entry[2][9]

		return ros_tm, x_mpc, y_mpc, v_mpc, psi_mpc, x_ref, y_ref, psi_ref, d_f_opt, acc_opt
end

function plot_mpc_trajs(fname, logdir)
	data_dict = load(@sprintf("%s/%s", logdir, fname))


	for key in sort(collect(keys(data_dict)), lt=(x,y)->compare_timestamp(x,y))
		f = figure(figsize=(8.4,8.8))
		tm, xm, ym, vm, pm, xr, yr, pr, df, acc = parse_dict_entry(data_dict[key])

		t_ref = collect(1:length(xm))
		v_ref = 15.0*ones(length(xm))

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
	end
end


if ! isinteractive()
	
	logdir = "/home/govvijay/catkin_ws/src/mkz_mpc_path_follower/data/tests_3_24/test2"
	fname = "mpc_np_2018-03-24T19:30:33.jld"
	plot_mpc_trajs(fname, logdir)
end

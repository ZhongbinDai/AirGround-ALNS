classdef Data
	properties
		N_d % number of drones per truck
		m   % number of trucks
		Q_t % truck capacity
		Q_d % drone capacity
		R_t % truck range
		R_d % drone range
		v_t % truck speed
		v_d % drone speed
		N   % number of nodes

		x   % x-coordinates (Nx1)
		y   % y-coordinates (Nx1)
		w   % weights (Nx1)

		tau_t % Truck travel time matrix (NxN)
		tau_d % Drone flight time matrix (NxN)
		dis_t % Truck distance matrix (NxN, Manhattan)
		dis_d % Drone distance matrix (NxN, Euclidean)
	end

	methods
		function obj = Data(datafile)
			fid = fopen(datafile, 'r');
			assert(fid ~= -1, '无法打开数据文件: %s', datafile);
			cleanup = onCleanup(@() fclose(fid));

			fgetl(fid);
			obj.N_d = str2double(strtrim(fgetl(fid)));
			fgetl(fid);
			obj.m   = str2double(strtrim(fgetl(fid)));
			fgetl(fid);
			obj.Q_t = str2double(strtrim(fgetl(fid)));
			fgetl(fid);
			obj.Q_d = str2double(strtrim(fgetl(fid)));
			fgetl(fid);
			obj.R_t = str2double(strtrim(fgetl(fid)));
			fgetl(fid);
			obj.R_d = str2double(strtrim(fgetl(fid)));
			fgetl(fid);
			obj.v_t = str2double(strtrim(fgetl(fid)));
			fgetl(fid);
			obj.v_d = str2double(strtrim(fgetl(fid)));
			fgetl(fid);
			obj.N   = str2double(strtrim(fgetl(fid)));
			fgetl(fid);

			obj.x = zeros(obj.N, 1);
			obj.y = zeros(obj.N, 1);
			obj.w = zeros(obj.N, 1);

			tokens = strsplit(strtrim(fgetl(fid)));
			obj.x(1) = str2double(tokens{1});
			obj.y(1) = str2double(tokens{2});
			fgetl(fid);

			for i = 2:obj.N
				tokens = strsplit(strtrim(fgetl(fid)));
				obj.x(i) = str2double(tokens{1});
				obj.y(i) = str2double(tokens{2});
				obj.w(i) = str2double(tokens{4});
			end

			fgetl(fid);

			obj.tau_t = zeros(obj.N, obj.N);
			obj.tau_d = zeros(obj.N, obj.N);
			obj.dis_t = zeros(obj.N, obj.N);
			obj.dis_d = zeros(obj.N, obj.N);

			obj = calculate(obj);
		end

		function obj = calculate(obj)
			for i = 1:obj.N
				for j = 1:obj.N
					obj.dis_t(i,j) = abs(obj.x(i) - obj.x(j)) + abs(obj.y(i) - obj.y(j));
					obj.dis_d(i,j) = sqrt((obj.x(i) - obj.x(j))^2 + (obj.y(i) - obj.y(j))^2);
					obj.tau_t(i,j) = obj.dis_t(i,j) / obj.v_t;
					obj.tau_d(i,j) = obj.dis_d(i,j) / obj.v_d;
				end
			end
		end
	end
end


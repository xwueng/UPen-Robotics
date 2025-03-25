function [master_eqtable, deriv_tables, master_coefs, cnstr_table] = getMasterTables(x, ncoef, nd, waypoints, wpdindex, S, T)
    
    % traj_time/S time from wp(1) to wp(i)
    % e.g: [0    3.4641    6.9282   10.3923   13.8564];
    
    % T/d0 segment travel time from seg start to seg end
    % T = [3.4641    3.4641    3.4641    3.4641];

    nseg = size(waypoints, 2)-1;

    mncol = ncoef*nseg;
    mnrow = mncol;
    meqt_index = 0;
    lastcol = mncol;

    syms x [nseg,1];
    
    % segscales stores each segment's scale 
    segscales = zeros(nseg, 1);
    
    deriv_tables = sym('D', [(nd+1)*2, mncol]);
    master_eqtable = sym('X', [mnrow, mncol]);
    coef_values = zeros(mnrow, 1);
    cnstr_table = zeros(mnrow, 3)

    c = sym("c", [ncoef,4])
    master_coefs = [c(:,1); c(:,2); c(:,3); c(:,4)];

    for i = 1:3
        cnstr_table(:, i) = getConstraints(mnrow, waypoints, i);

    for i= 1:mnrow
        for j = 1:mncol
           master_eqtable(i,j) = 0;
        end
    end

 
    % dcol = size(deriv_table, 2);
    dcol = 8;
    drow = (nd+1)*2;


    % 1. insert 8 waypoints equations into master equations table
    for i = 1:nseg
        deriv_tables(1:drow, i*8-7:i*8) = getPolyMatrix(x(i), ncoef, nd);
        master_eqtable(i*2-1:i*2, i*8-7:i*8) = deriv_tables(1:2, i*8-7:i*8);
        meqt_index = 2*i;
        segscales(i) = (S(i+1) - S(i))/T(i);  
        % segscales(i) = S(i+1);  
    end


    % 2. endpoints' derivatives are zeros: insert derivative orders 1-3 equations into master equations
    % table: total 6
    for i = 1:3

        % seg1: first derivative is in deriv_tables 4th row, 
        % 2nd deriv in 6th col 1-8
        meqt_index = meqt_index + 1;
        master_eqtable(meqt_index, 1:8) = deriv_tables((i+1)*2, 1:8); 
        
        % endpoint seg 4: first derivative is in deriv_tables 4th row, 
        % col lastcol-ncoef+1:lastcol 
        meqt_index = meqt_index + 1;
        master_eqtable(meqt_index, lastcol-ncoef+1:lastcol) = deriv_tables((i+1)*2, lastcol-ncoef+1:lastcol); 
    end

    % 3. Continuity: for each derive, a seg's derivative value at seg end ==
    % next seg's derivative value at start
    %  p1(S2)=p2(S2), p2(S3) = p3(S3), p3(S4)=p4(S4)


    for i = 1:nseg-1 % i: segs' intermediate points

        %x(i) is time variable t: (S2-S1)/T2, (S3-S2)/T3, (S4-S3)/T4,  all =1 
        xivalue = (S(i+1) - S(i))/T(i);  

        %x(i+1) is time variable t: i+1=3: (S2-S2)/T3, i+1=4: (S3-S3)/T4, i+1=5: (S4-S4)/T5 all = 0
        xip1value = (S(i+1) - S(i+1))/T(i+1); 

        for j = 1:nd % j: order of derivatives
            meqt_index = meqt_index + 1;
            master_eqtable(meqt_index, i*8-7:i*8) = ...
                deriv_tables((j+1)*2, i*8-7:i*8) - deriv_tables((j+1)*2, i*8+1:(i+1)*8); 
            master_eqtable(meqt_index, i*8-7:i*8) = ...
                subs(master_eqtable(meqt_index, i*8-7:i*8) , [x(i), x(i+1)], [xivalue, xip1value]);
            
        end % j: order of derivatives
    end  % i: segs' intermediate points

   % xip1value = (S(i+1) - S(i+1))/T(i+1) = 1 
   master_eqvalues = subs(master_eqtable, x, segscales)
   coef_values = solve(master_eqvalues * master_coefs == cnstr_table(:, wpdindex)) 
    end % loop i   


end % getMasterTables()

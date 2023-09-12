function write_transform(fix_T_moving, outputfile)
    fid = fopen(outputfile, 'w');
    fprintf(fid, '%.9f %.9f %.9f %.9f', fix_T_moving(1,1), fix_T_moving(1,2), fix_T_moving(1,3), fix_T_moving(1,4));
    fprintf(fid, ' ');
    fprintf(fid, '%.9f %.9f %.9f %.9f', fix_T_moving(2,1), fix_T_moving(2,2), fix_T_moving(2,3), fix_T_moving(2,4));
    fprintf(fid, ' ');
    fprintf(fid, '%.9f %.9f %.9f %.9f', fix_T_moving(3,1), fix_T_moving(3,2), fix_T_moving(3,3), fix_T_moving(3,4));
    fprintf(fid, '\n');
    fclose(fid);
end

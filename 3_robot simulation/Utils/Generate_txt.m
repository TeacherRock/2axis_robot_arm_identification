function Generate_txt(Data, FilePath)

File = fopen(FilePath, 'wt');

for i = 1 : size(Data, 1)
    for j = 1 : size(Data, 2)
        fprintf(File, '%f', Data(i, j));
        if j ~= size(Data, 2)
            fprintf(File, '\t');
        end
    end
    if i ~= size(Data, 1)
       fprintf(File, '\n');
    end
end

fclose(File);

StrDisp(['Generate ', char(FilePath), 'Successfully ']);

end
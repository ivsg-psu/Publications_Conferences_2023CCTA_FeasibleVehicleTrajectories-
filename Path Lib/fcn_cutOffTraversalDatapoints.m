function [traversal] = fcn_cutOffTraversalDatapoints(reference_traversal,cut_off_length)
rows_to_select_RCL = find(reference_traversal.Station >= cut_off_length);
traversal.X = reference_traversal.X(rows_to_select_RCL);
traversal.Y = reference_traversal.Y(rows_to_select_RCL);
traversal.Z = reference_traversal.Z(rows_to_select_RCL);
traversal.Station = reference_traversal.Station(rows_to_select_RCL);
end
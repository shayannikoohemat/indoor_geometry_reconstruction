// batch processing thinlaser 
// for information about the code read this thread: https://stackoverflow.com/questions/5642021/batch-process-all-files-in-directory
// %%~dpnxF expands to Drive, P‎ath, base‎Name and e‎X‎tension of the current file

pushd D:\Mapping\Tools\indoor_geometry_reocnstruction\Debug
for %%F in (E:\publication_data\FB_dataset\data\data\process_segmentation\*.laser) do (
	indoor_reconstruction -i %%~dpnxF -o E:\publication_data\FB_dataset\data\data\process_segmentation\out\%%~nxF -maxd_p 0.10 -maxd_s 0.12
)
popd




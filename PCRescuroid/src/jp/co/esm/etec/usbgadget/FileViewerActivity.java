package jp.co.esm.etec.usbgadget;

import java.io.File;
import java.io.FileFilter;

import android.app.Activity;
import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.View;
import android.webkit.MimeTypeMap;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemClickListener;
import android.widget.ArrayAdapter;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

public class FileViewerActivity extends Activity implements OnItemClickListener {
    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        initialize();
    }
    
    private void initialize() {
    	ListView listView = (ListView)this.findViewById(R.id.file_list_view);
    	ArrayAdapter<FileInfo> adapter = new ArrayAdapter<FileInfo>(this, android.R.layout.simple_list_item_1);
    	Intent intent = getIntent();
    	final boolean existData = intent != null && intent.getData() != null;
    	if (!existData) {
    		adapter.add(new FileInfo(Environment.getExternalStorageDirectory()));
        	adapter.add(new FileInfo(Environment.getRootDirectory()));
    	} else {
    		File file = new File(intent.getData().getPath());
    		FileFilter filter = new ExtensionFileFileter("iso");
	    	for (File f : file.listFiles(filter)) {
	    		adapter.add(new FileInfo(f));
	    	}
    	}
    	listView.setAdapter(adapter);
    	listView.setOnItemClickListener(this);
    	
    	TextView textView = (TextView)findViewById(R.id.current_path);
    	textView.setText(existData ? intent.getData().getPath() : null);
    }

	@Override
	public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
		FileInfo info = (FileInfo)parent.getItemAtPosition(position);
		File clickedFile = info.getFile();
		if (!clickedFile.canRead()) {
			Toast.makeText(this, clickedFile.getName() + " cannot read.", Toast.LENGTH_SHORT);
			return;
		}
		if (clickedFile.isDirectory()) {
			Intent intent = new Intent(Intent.ACTION_VIEW);
			intent.setDataAndType(Uri.fromFile(clickedFile), "text/directory");
			Log.d("DEBUG...", "intent:" + intent);
			startActivity(intent);
			
		} else {
			Intent intent = new Intent(Intent.ACTION_VIEW);
			Uri uri = Uri.fromFile(clickedFile);
			String path = uri.getPath();
			String ext = null;//MimeTypeMap.getFileExtensionFromUrl(path);
			if (ext == null || ext.length() == 0) {
				ext = getExtension(path);
			}
			String mimeType = MimeTypeMap.getSingleton().getMimeTypeFromExtension(ext.toLowerCase());
			intent.setDataAndType(uri, mimeType);
			Log.d("DEBUG...", "intent:" + intent);
			setResult(RESULT_OK, intent);
			finish();
			//startActivity(intent);
		}
	}
    
    private String getExtension(String path) {
    	int index = path.lastIndexOf('.');
    	if (index < 0) {
    		return "";
    	}
    	return path.substring(index + 1, path.length());
    }
    
    private static class ExtensionFileFileter implements FileFilter {
    	
    	private String extension;
    	
    	public ExtensionFileFileter(String extension) {
    		this.extension = extension;
    	}

		@Override
		public boolean accept(File file) {
			if (extension == null) {
				return true;
			}
			if (file.isDirectory()) {
				return true;
			}
			return file.getName().endsWith("." + extension);
		}
    	
    }
    
    private static class FileInfo {
    	private File file;
    	
    	public FileInfo(File file) {
    		this.file = file;
    	}
    	
    	public File getFile() {
    		return file;
    	}
    	
    	public String getName() {
    		return file.getName();
    	}
    	
    	@Override
    	public String toString() {
    		return file.getName();
    	}
    }
}
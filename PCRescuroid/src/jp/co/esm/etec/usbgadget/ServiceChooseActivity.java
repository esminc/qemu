package jp.co.esm.etec.usbgadget;

import java.io.File;

import android.app.Activity;
import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.View;

/**
 * ISOファイルを提供するサービスを選択する画面です。
 */
public class ServiceChooseActivity extends Activity {
	
	private static final int REQUEST_LOCAL = 0;
	
	public static final String EXTRA_SERVICE_NAME = "service_name";

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.service_choose);
	}

	public void chooseLocalFile(View v) {
		Intent intent = new Intent(Intent.ACTION_VIEW);
		intent.addCategory(Intent.CATEGORY_DEFAULT);
		File sd = Environment.getExternalStorageDirectory();
		intent.setDataAndType(Uri.fromFile(sd), "text/directory");
		Log.d("DEBUG...", "intent:" + intent);
		startActivityForResult(intent, REQUEST_LOCAL);
	}
	
	public void chooseDropboxFile(View v) {
		
	}

	/* (non-Javadoc)
	 * @see android.app.Activity#onActivityResult(int, int, android.content.Intent)
	 */
	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {
		if (REQUEST_LOCAL == requestCode) {
			if (data != null) {
				data.putExtra(EXTRA_SERVICE_NAME, "SD");				
			}
			setResult(resultCode, data);
			finish();
		}
	}
	
	
}

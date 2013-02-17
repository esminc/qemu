package jp.co.esm.etec.usbgadget;

import android.app.Activity;
import android.app.AlertDialog;
import android.app.Dialog;
import android.content.DialogInterface;
import android.content.DialogInterface.OnClickListener;
import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

/**
 * ISOファイルのマウント、アンマウントを行うアクティビティです。
 */
public class ISOFileMounter extends Activity {
	
	private static final String TAG = "ISOFileMounter";
	
	private static final int DIALOG_MOUNT_ERORR_ID = 1;
	
	private static final int DIALOG_UNMOUNT_ERORR_ID = 2;
	
	private static final int REQUEST_FILE_CHOOSE = 1;
	
	private String isoFilePath;

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.iso_file_mounter);
	}
	
	/**
	 * ファイルを
	 * @param view
	 */
	public void fileChoose(View view) {
		Intent intent = new Intent();
		intent.setClass(getApplicationContext(), ServiceChooseActivity.class);
		startActivityForResult(intent, REQUEST_FILE_CHOOSE);
	}
	
	/* (non-Javadoc)
	 * @see android.app.Activity#onActivityResult(int, int, android.content.Intent)
	 */
	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {
		if (REQUEST_FILE_CHOOSE == requestCode) {
			if (data != null) {
				Uri fileUri = data.getData();
				isoFilePath = fileUri.getPath();
				String serviceName = data.getStringExtra(ServiceChooseActivity.EXTRA_SERVICE_NAME);
				TextView fileNameView = (TextView)findViewById(R.id.file_name);
				fileNameView.setText(isoFilePath);
				TextView serviceNameView = (TextView)findViewById(R.id.service_name);
				serviceNameView.setText(serviceName);
			}
		}
	}

	/**
	 * {@link View}から呼び出されます。ISOファイルのマウント・アンマウントを行います。
	 * 
	 * @param v {@link ToggleButton}
	 */
	public void mount(View v) {
		Log.v(TAG, "mount()");
		ToggleButton mountSwitch = (ToggleButton)v;
		if (mountSwitch.isChecked()) {
			try {
				mount();
				Toast.makeText(this, "mount", Toast.LENGTH_LONG).show();
			} catch (UnsupportedOperationException e) {
				showDialog(DIALOG_MOUNT_ERORR_ID);
				mountSwitch.setChecked(false);
			}
		} else {
			try {
				unmount();
				Toast.makeText(this, "unmount", Toast.LENGTH_LONG).show();
			} catch (UnsupportedOperationException e) {
				showDialog(DIALOG_UNMOUNT_ERORR_ID);
				mountSwitch.setChecked(true);
			}
		}
	}
	
	/* (non-Javadoc)
	 * @see android.app.Activity#onCreateDialog(int)
	 */
	@Override
	protected Dialog onCreateDialog(int id) {
		AlertDialog.Builder builder = new AlertDialog.Builder(this);
		if (id == DIALOG_MOUNT_ERORR_ID) {
			builder.setMessage("Cannot mount.");
		} else if (id == DIALOG_UNMOUNT_ERORR_ID) {
			builder.setMessage("Cannot unmount.");
		}
		builder.setNeutralButton("OK", new OnClickListener() {
			
			@Override
			public void onClick(DialogInterface dialog, int which) {
				dialog.dismiss();
			}
		});
		return builder.create();
	}
	
	private MountService getMountService() {
		return MountServiceUtil.getMountService(this.getApplicationContext());
	}
	
	private void mount() {
		if (isoFilePath != null) {
			MountService mountService = getMountService();
			mountService.mountMedia(isoFilePath);
		}
	}

	private void unmount() {
		if (isoFilePath != null) {
			MountService mountService = getMountService();
			mountService.unmountMedia(isoFilePath);
		}
	}
}

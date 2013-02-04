package jp.co.esm.etec.usbgadget;

import android.app.Activity;
import android.app.AlertDialog;
import android.app.Dialog;
import android.content.DialogInterface;
import android.content.DialogInterface.OnClickListener;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Toast;
import android.widget.ToggleButton;

/**
 * SDカードのマウント、アンマウントを行うアクティビティです。
 */
public class SDCardMounter extends Activity {
	
	private static final String TAG = "SDCardMounter";
	
	private static final int DIALOG_MOUNT_ERORR_ID = 1;
	
	private static final int DIALOG_UNMOUNT_ERORR_ID = 2;

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.iso_file_mounter);
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
		MountService mountService = getMountService();
		mountService.setMassStorageEnabled(true);
	}

	private void unmount() {
		MountService mountService = getMountService();
		mountService.setMassStorageEnabled(false);
	}
}

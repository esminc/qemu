package jp.co.esm.etec.usbgadget;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;

/**
 * SDカードのマウントとISOファイルのマウントを選択する画面です。
 */
public class FormatChooseActivity extends Activity {

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.format_choose);
	}
	
	/**
	 * SDカードをマウントする{@link Activity}を起動します。
	 * @param view SDカードボタン
	 */
	public void chooseSD(View view) {
		Intent intent = new Intent();
		intent.setClass(getApplicationContext(), SDCardMounter.class);
		startActivity(intent);
	}

	/**
	 * ISOファイルをマウントする{@link Activity}を起動します。
	 * @param view ISOファイルボタン
	 */
	public void chooseISO(View view) {
		Intent intent = new Intent();
		intent.setClass(getApplicationContext(), ISOFileMounter.class);
		startActivity(intent);
	}
}

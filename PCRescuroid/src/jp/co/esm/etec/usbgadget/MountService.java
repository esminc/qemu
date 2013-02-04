package jp.co.esm.etec.usbgadget;


/**
 * USBストレージ内のISOファイルをUSBホストに対してマウント・アンマウントを行うサービスのインタフェースです。
 */
public interface MountService {
	/**
	 * ISOイメージとしてマウントするモード
	 */
	public static final String MODE_CD = "ums_mode_cd"; 
	/**
	 * SDカードとしてマウントするモード
	 */
	public static final String MODE_SD = "ums_mode_sd"; 
	/**
	 * 指定のパスのISOファイルがマウントされている場合に{@code true}を返します。
	 * 
	 * @param path ISOファイルのパス
	 * @return　マウントされている場合は{@code true}、マウントされていない場合は{@code false}
	 * @throws UnsupportedOperationException プラットフォームが操作をサポートしない場合
	 */
	public boolean isISOFileMounted(String path);
	
	/**
	 * 指定のパスのISOファイルをマウントします。
	 * 
	 * @param path マウントするISOファイルのパス
	 * @throws UnsupportedOperationException プラットフォームが操作をサポートしない場合
	 */
	public void mountMedia(String path);

	/**
	 * 指定のパスのISOファイルをアンマウントします。
	 * 
	 * @param path アンマウントするISOファイルのパス
	 * @throws UnsupportedOperationException プラットフォームが操作をサポートしない場合
	 */
	public void unmountMedia(String path);
	
	
	public boolean getMassStorageConnected();

	 public boolean getMassStorageEnabled();
	
	 public void setMassStorageEnabled(boolean enable);

	/**
	 * ストレージのモードを取得します。
	 * 
	 * @return {@link #MODE_CD}または{@link #MODE_SD}
	 */
	public String getMassStorageMode();
	
	/**
	 * ストレージのモードを設定します。
	 * 
	 * @param mode {@link #MODE_CD}または{@link #MODE_SD}
	 */
	public void setMassStorageMode(String mode);
}
